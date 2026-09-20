function candidate = encodeScenarioGains(scenario)
%ENCODESCENARIOGAINS Encode a feasible scenario controller for particleswarm.
%
% This is the inverse of applyScenarioGains for one controller mode. It is
% used to seed optimization with known manual/optimized controller settings.

if ~isstruct(scenario) || ~isfield(scenario, 'controller')
    error('agc:opt:encodeScenarioGains:Scenario', 'scenario must contain a controller struct.');
end
controller = scenario.controller;
positive = [diag(controller.KR).', diag(controller.Kxi).', diag(controller.Lambda).', ...
    controller.kd, controller.ks];
validateattributes(positive, {'numeric'}, {'real', 'finite', 'positive', 'numel', 14});
validateattributes(controller.alpha, {'numeric'}, {'real', 'finite', '>', 0, '<', 1, 'scalar'});

candidate = [log10(positive), controller.alpha];
mode = lower(char(string(controller.mode)));
if strcmp(mode, 'euclidean')
    validateattributes(controller.gammaE, {'numeric'}, {'real', 'finite', 'positive', 'size', [10, 1]});
    candidate = [candidate, log10(controller.gammaE(:).')];
elseif strcmp(mode, 'bregman')
    validateattributes(controller.gammaB, {'numeric'}, {'real', 'finite', 'positive', 'scalar'});
    candidate = [candidate, log10(controller.gammaB)];
elseif ~strcmp(mode, 'nominal')
    error('agc:opt:encodeScenarioGains:Mode', 'Unsupported controller mode: %s.', mode);
end
end
