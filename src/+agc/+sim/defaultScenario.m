function scenario = defaultScenario(replayId, mode, coriolis, duration)
%DEFAULTSCENARIO Reproducible paper-validation scenario from one replay artifact.
if nargin < 4 || isempty(duration), duration = 30; end
filePath = fullfile(agc.io.repositoryRoot(), 'trajectories', 'processed', char(replayId) + ".mat");
if ~isfile(filePath)
    error('agc:sim:defaultScenario:ReplayNotFound', 'Replay artifact not found: %s', filePath);
end
trajectory = agc.sim.replayTrajectory(filePath);
desired0 = trajectory(0);
m = 3.646; cog = [0; 0; -0.00229];
I = [0.04092; 0.04017; 0.06921; 5.656e-5; 1.313e-5; -6.494e-5];
pi = [m; m * cog; I];
initialH = desired0.H; initialH(1:3,4) = initialH(1:3,4) + [0.2; -0.1; 0.15];
controller = struct('mode', char(mode), 'coriolis', char(coriolis), ...
    'KR', diag([4, 5, 6]), 'Kxi', diag([3, 3, 4]), 'Lambda', diag(2 * ones(6,1)), ...
    'kd', 1, 'ks', 0.5, 'alpha', 0.5, 'gravity', [0; 0; -9.81], ...
    'Gamma', 1e-3 * eye(10), 'gammaB', 1e-3);
estimate = pi;
if strcmpi(mode, 'bregman'), estimate = agc.math.pseudoFromPi(pi); end
scenario = struct('plantPi', pi, 'initial', struct('H', initialH, 'V', desired0.V), ...
    'trajectory', trajectory, 'duration', duration, 'dtPlant', 0.002, ...
    'dtControl', 0.01, 'dtAdaptation', 0.01, 'controller', controller, ...
    'initialEstimate', estimate);
end
