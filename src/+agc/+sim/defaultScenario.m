function scenario = defaultScenario(replayId, mode, coriolis, duration, gainSource)
%DEFAULTSCENARIO Reproducible paper-validation scenario from one replay artifact.
if nargin < 4 || isempty(duration), duration = 30; end
if nargin < 5 || isempty(gainSource), gainSource = 'optimized'; end
filePath = fullfile(agc.io.repositoryRoot(), 'trajectories', 'processed', char(replayId) + ".mat");
if ~isfile(filePath)
    error('agc:sim:defaultScenario:ReplayNotFound', 'Replay artifact not found: %s', filePath);
end
trajectory = agc.sim.replayTrajectory(filePath);
desired0 = trajectory(0);
m = 3.646; cog = [0; 0; -0.00229];
I = [0.04092; 0.04017; 0.06921; 5.656e-5; 1.313e-5; -6.494e-5];
pi = [m; m * cog; I];
% A deliberately substantial, asymmetric temporary payload exposes the
% post-release model mismatch without placing the added mass at the CoG.
payload = struct('mass', 0.75, 'dimensions', [0.12; 0.12; 0.08], ...
    'center', [0.20; 0.05; -0.12]);
payloadDrop = struct('releaseTime', 10, 'barePi', pi, ...
    'loadedPi', agc.plant.compoundPi(pi, payload), 'payload', payload);
initialH = desired0.H; initialH(1:3,4) = initialH(1:3,4) + [0.2; -0.1; 0.15];
gains = selectGains(gainSource, mode, coriolis);

% The paper gravity wrench uses an upward compensation vector; Robotics uses
% the physical downward world acceleration stored separately as plantGravity.
controller = struct('mode', char(mode), 'coriolis', char(coriolis), ...
    'KR', diag(gains.KRdiag), 'Kxi', diag(gains.Kxidiag), 'Lambda', diag(gains.LambdaDiag), ...
    'kd', gains.kd, 'ks', gains.ks, 'alpha', gains.alpha, 'gravity', [0; 0; 9.81], ...
    'gammaE', gains.gammaE, 'gammaB', gains.gammaB);

% Every controller starts from the exact loaded model. The nominal and
% Euclidean modes use pi coordinates, while Bregman retains pseudo-inertia.
estimate = payloadDrop.loadedPi;
if strcmpi(mode, 'bregman')
    estimate = agc.math.pseudoFromPi(estimate);
end
scenario = struct('plantPi', pi, 'initial', struct('H', initialH, 'V', desired0.V), ...
    'trajectory', trajectory, 'duration', duration, 'dtPlant', 0.002, ...
    'dtControl', 0.02, 'dtAdaptation', 0.01, 'plantGravity', [0; 0; -9.81], 'controller', controller, ...
    'initialEstimate', estimate, 'payloadDrop', payloadDrop);
end

function gains = selectGains(source, mode, coriolis)
%SELECTGAINS Load a named repository gain registry.
switch lower(char(string(source)))
    case 'manual'
        gains = manual_gains(mode, coriolis);
    case 'optimized'
        gains = optimized_gains(mode, coriolis);
    otherwise
        error('agc:sim:defaultScenario:UnknownGainSource', ...
            'gainSource must be ''manual'' or ''optimized''.');
end
end
