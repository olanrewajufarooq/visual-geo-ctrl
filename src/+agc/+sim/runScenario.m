function run = runScenario(scenario)
%RUNSCENARIO Run one deterministic, multi-rate closed-loop simulation.
%
% Controller, estimator, and Robotics plant use explicit fixed rates. The
% controller wrench is zero-order-held between controller sample instants.

%% Scenario contract and Robotics Toolbox plant

validateScenario(scenario);

% plantGravity is physical world acceleration. controller.gravity is the
% separate compensation convention used by the paper controller.
plant = agc.plant.floatingBody(scenario.plantPi, scenario.plantGravity);

%% Preallocate the complete replay log
nSteps = round(scenario.duration / scenario.dtPlant);
n = nSteps + 1;
run = struct('t', (0:nSteps).' * scenario.dtPlant, 'H', zeros(4,4,n), ...
    'V', zeros(n,6), 'Hdesired', zeros(4,4,n), 'Vdesired', zeros(n,6), ...
    'wrench', zeros(n,6), 's', zeros(n,6), ...
    'Psi', zeros(n,1), 'Vs', zeros(n,1), 'minPseudoEigenvalue', nan(n,1), ...
    'mode', char(scenario.controller.mode), 'coriolis', char(scenario.controller.coriolis));
state = scenario.initial;
estimate = scenario.initialEstimate;
controlEvery = round(scenario.dtControl / scenario.dtPlant);
adaptEvery = round(scenario.dtAdaptation / scenario.dtPlant);
lastWrench = zeros(6,1);
lastDiagnostics = emptyDiagnostics();

%% Fixed-step plant loop
for k = 1:n
    t = run.t(k);
    desired = scenario.trajectory(t);
    % Evaluate and hold the wrench only at controller sample instants.
    if mod(k - 1, controlEvery) == 0
        [lastWrench, lastDiagnostics] = agc.paper.controller( ...
            state, desired, scenario.controller, estimate, []);
    end
    if mod(k - 1, adaptEvery) == 0 && ~strcmpi(scenario.controller.mode, 'nominal')
        [~, ~, estimate] = agc.paper.controller( ...
            state, desired, scenario.controller, estimate, scenario.dtAdaptation);
    end
    % Log the state that corresponds to the wrench before plant propagation.
    run.H(:,:,k) = state.H;
    run.V(k,:) = state.V(:).';
    run.Hdesired(:,:,k) = desired.H;
    run.Vdesired(k,:) = desired.V(:).';
    run.wrench(k,:) = lastWrench.';
    run.s(k,:) = lastDiagnostics.s.';
    run.Psi(k) = lastDiagnostics.Psi;
    run.Vs(k) = lastDiagnostics.Vs;
    if strcmpi(scenario.controller.mode, 'bregman')
        run.minPseudoEigenvalue(k) = min(eig(0.5 * (estimate + estimate.')));
    end
    if k < n
        state = agc.plant.propagate(plant, state, lastWrench, scenario.dtPlant);
    end
end
run.finalEstimate = estimate;
end

function validateScenario(s)
%VALIDATESCENARIO Validate fields and rate alignment before simulation.
required = {'plantPi', 'initial', 'trajectory', 'duration', 'dtPlant', ...
    'dtControl', 'dtAdaptation', 'plantGravity', 'controller', 'initialEstimate'};
for k = 1:numel(required)
    if ~isfield(s, required{k})
        error('agc:sim:runScenario:MissingField', 'scenario.%s is required.', required{k});
    end
end
validateattributes(s.duration, {'numeric'}, {'real', 'finite', 'positive', 'scalar'});
for value = [s.dtPlant, s.dtControl, s.dtAdaptation]
    validateattributes(value, {'numeric'}, {'real', 'finite', 'positive', 'scalar'});
    ratio = value / s.dtPlant;
    if abs(ratio - round(ratio)) > 1e-10
        error('agc:sim:runScenario:RateRatio', 'All rates must be integer multiples of dtPlant.');
    end
end
if abs(s.duration / s.dtPlant - round(s.duration / s.dtPlant)) > 1e-10
    error('agc:sim:runScenario:DurationRatio', 'duration must be an integer multiple of dtPlant.');
end
end

function diagnostics = emptyDiagnostics()
%EMPTYDIAGNOSTICS Supply well-defined values before the first control tick.
diagnostics = struct('s', zeros(6,1), 'Psi', 0, 'Vs', 0);
end
