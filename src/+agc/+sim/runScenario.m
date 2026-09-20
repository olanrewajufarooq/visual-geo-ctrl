function run = runScenario(scenario)
%RUNSCENARIO Run one deterministic, multi-rate closed-loop simulation.
%
% Controller, estimator, and Robotics plant use explicit fixed rates. The
% controller wrench is zero-order-held between controller sample instants.

%% Scenario contract and Robotics Toolbox plant

validateScenario(scenario);

% plantGravity is physical world acceleration. controller.gravity is the
% separate compensation convention used by the paper controller.
[loadedPlant, barePlant, payloadDrop] = configuredPlants(scenario);

%% Preallocate the complete replay log
nSteps = round(scenario.duration / scenario.dtPlant);
n = nSteps + 1;
run = struct('t', (0:nSteps).' * scenario.dtPlant, 'H', zeros(4,4,n), ...
    'V', zeros(n,6), 'Hdesired', zeros(4,4,n), 'Vdesired', zeros(n,6), ...
    'wrench', zeros(n,6), 's', zeros(n,6), 'activePlantPi', zeros(n,10), ...
    'Psi', zeros(n,1), 'Vs', zeros(n,1), 'estimatePi', zeros(n,10), ...
    'minPseudoEigenvalue', nan(n,1), ...
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
    [plant, activePlantPi] = activePlantAtTime(loadedPlant, barePlant, payloadDrop, t, scenario.plantPi);
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
    run.activePlantPi(k,:) = activePlantPi.';
    run.estimatePi(k,:) = estimateToPi(scenario.controller.mode, estimate).';
    if strcmpi(scenario.controller.mode, 'bregman')
        run.minPseudoEigenvalue(k) = min(eig(0.5 * (estimate + estimate.')));
    end
    if k < n
        state = agc.plant.propagate(plant, state, lastWrench, scenario.dtPlant);
    end
end
run.finalEstimate = estimate;
end

function [loadedPlant, barePlant, payloadDrop] = configuredPlants(scenario)
%CONFIGUREDPLANTS Build fixed or hybrid payload-drop plant representations.

barePlant = agc.plant.floatingBody(scenario.plantPi, scenario.plantGravity);
loadedPlant = barePlant;
payloadDrop = [];
if ~isfield(scenario, 'payloadDrop'), return; end

payloadDrop = scenario.payloadDrop;
loadedPlant = agc.plant.floatingBody(payloadDrop.barePi, scenario.plantGravity, payloadDrop.payload);
end

function [plant, pi] = activePlantAtTime(loadedPlant, barePlant, payloadDrop, time, barePi)
%ACTIVEPLANTATTIME Select the no-impulse plant state at the release boundary.

if isempty(payloadDrop)
    plant = barePlant;
    pi = barePi;
elseif time < payloadDrop.releaseTime
    plant = loadedPlant;
    pi = payloadDrop.loadedPi;
else
    plant = barePlant;
    pi = payloadDrop.barePi;
end
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
if isfield(s, 'payloadDrop')
    validatePayloadDrop(s);
end
end

function validatePayloadDrop(s)
%VALIDATEPAYLOADDROP Ensure the controller and plant share one hybrid model.

drop = s.payloadDrop;
required = {'releaseTime', 'barePi', 'loadedPi', 'payload'};
if ~isstruct(drop) || ~all(isfield(drop, required))
    error('agc:sim:runScenario:PayloadDrop', ...
        'payloadDrop must define releaseTime, barePi, loadedPi, and payload.');
end
validateattributes(drop.releaseTime, {'numeric'}, {'real', 'finite', 'positive', 'scalar'});
if drop.releaseTime >= s.duration
    error('agc:sim:runScenario:PayloadDropTime', ...
        'payloadDrop.releaseTime must be strictly before scenario.duration.');
end
if abs(drop.releaseTime / s.dtPlant - round(drop.releaseTime / s.dtPlant)) > 1e-10
    error('agc:sim:runScenario:PayloadDropAlignment', ...
        'payloadDrop.releaseTime must align with dtPlant.');
end
for value = {drop.barePi, drop.loadedPi}
    validateattributes(value{1}, {'numeric'}, {'real', 'finite', 'numel', 10});
end
if norm(drop.barePi(:) - s.plantPi(:)) > 1e-12
    error('agc:sim:runScenario:PayloadDropBarePi', ...
        'payloadDrop.barePi must match scenario.plantPi.');
end
derivedLoadedPi = agc.plant.compoundPi(drop.barePi, drop.payload);
if norm(drop.loadedPi(:) - derivedLoadedPi) > 1e-12
    error('agc:sim:runScenario:PayloadDropLoadedPi', ...
        'payloadDrop.loadedPi must match the physical payload composition.');
end
if ~agc.math.isSPD(agc.math.pseudoFromPi(drop.loadedPi))
    error('agc:sim:runScenario:PayloadDropPhysical', ...
        'payloadDrop.loadedPi must be physically consistent.');
end
end

function diagnostics = emptyDiagnostics()
%EMPTYDIAGNOSTICS Supply well-defined values before the first control tick.
diagnostics = struct('s', zeros(6,1), 'Psi', 0, 'Vs', 0);
end

function piHat = estimateToPi(mode, estimate)
%ESTIMATETOPI Convert the mode-specific estimator state for run logging.
if strcmpi(mode, 'bregman')
    piHat = agc.math.piFromPseudo(estimate);
else
    piHat = estimate(:);
end
end
