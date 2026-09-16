function [cost, detail] = objective(candidate, baseScenarios, decode, weights, useParallel)
%OBJECTIVE Evaluate explicit tracking, attitude, effort, and failure terms.
if nargin < 4 || isempty(weights), weights = struct(); end
if nargin < 5 || isempty(useParallel), useParallel = true; end
weights = defaults(weights);
scenarios = decode(candidate, baseScenarios);
if ~iscell(scenarios), scenarios = {scenarios}; end
batch = agc.batch.runBatch(scenarios, struct('parallel', useParallel));
cost = 0;
valid = 0;
for k = 1:numel(batch.metrics)
    if isempty(batch.failures{k})
        m = batch.metrics{k};
        cost = cost + weights.position * m.positionRMSE + ...
            weights.attitude * m.attitudeRMSE + weights.effort * m.wrenchRMS;
        valid = valid + 1;
    else
        cost = cost + weights.failure;
    end
end
if valid > 0, cost = cost / valid; end
detail = struct('batch', batch, 'failed', batch.successCount ~= numel(scenarios), ...
    'candidate', candidate, 'weights', weights);
end

function weights = defaults(weights)
fallback = struct('position', 1, 'attitude', 1, 'effort', 0.01, 'failure', 1e6);
names = fieldnames(fallback);
for k = 1:numel(names)
    if ~isfield(weights, names{k}), weights.(names{k}) = fallback.(names{k}); end
end
end
