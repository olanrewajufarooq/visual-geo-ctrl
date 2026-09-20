function [cost, detail] = objective(candidate, baseScenarios, decode, weights, useParallel)
%OBJECTIVE Evaluate weighted tracking, effort, and failure costs.

%% Decode one optimizer candidate into immutable scenarios

if nargin < 4 || isempty(weights), weights = struct(); end
if nargin < 5 || isempty(useParallel), useParallel = true; end
weights = defaults(weights);
scenarios = decode(candidate, baseScenarios);
if ~iscell(scenarios), scenarios = {scenarios}; end
batch = agc.batch.runBatch(scenarios, struct('parallel', useParallel));

%% Accumulate the explicit, documented optimization objective

cost = 0;
valid = 0;
for k = 1:numel(batch.metrics)
    if isempty(batch.failures{k})
        m = batch.metrics{k};
        cost = cost + weights.position * m.positionRMSE + ...
            weights.attitude * m.attitudeRMSE + weights.effort * m.wrenchRMS + ...
            weights.estimation * m.parameterEstimationRMSE;
        valid = valid + 1;
    else
        cost = cost + weights.failure;
    end
end
% Average successful runs so a batch size change does not rescale the cost.
if valid > 0, cost = cost / valid; end
detail = struct('batch', batch, 'failed', batch.successCount ~= numel(scenarios), ...
    'candidate', candidate, 'weights', weights);
end

function weights = defaults(weights)
%DEFAULTS Fill omitted objective weights without overriding user choices.
fallback = struct('position', 1, 'attitude', 1, 'effort', 0.01, 'estimation', 0.1, 'failure', 1e6);
names = fieldnames(fallback);
for k = 1:numel(names)
    if ~isfield(weights, names{k}), weights.(names{k}) = fallback.(names{k}); end
end
end
