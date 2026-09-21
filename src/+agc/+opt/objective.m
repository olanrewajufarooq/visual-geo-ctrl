function [cost, detail] = objective(candidate, baseScenarios, decode, weights, useParallel)
%OBJECTIVE Evaluate weighted tracking, effort, and failure costs.

%% Decode one optimizer candidate into immutable scenarios

if nargin < 4 || isempty(weights), weights = struct(); end
if nargin < 5 || isempty(useParallel), useParallel = true; end
weights = defaults(weights);
scales = agc.opt.objectiveScales();
scenarios = decode(candidate, baseScenarios);
if ~iscell(scenarios), scenarios = {scenarios}; end
batch = agc.batch.runBatch(scenarios, struct('parallel', useParallel));

%% Accumulate the explicit, documented optimization objective

cost = 0;
valid = 0;
for k = 1:numel(batch.metrics)
    if isempty(batch.failures{k})
        m = batch.metrics{k};
        cost = cost + weights.position * (m.positionRMSE / scales.position)^2 + ...
            weights.attitude * (m.attitudeRMSE / scales.attitude)^2 + ...
            weights.mass * (m.massEstimationRMSE / scales.mass)^2 + ...
            weights.cog * (m.centerOfMassEstimationRMSE / scales.cog)^2 + ...
            weights.linVel * (m.linearVelocityRMSE / scales.linVel)^2 + ...
            weights.angVel * (m.angularVelocityRMSE / scales.angVel)^2 + ...
            weights.inertia * (m.inertiaEstimationRMSE / scales.inertia)^2 + ...
            weights.effort * (m.wrenchRMS / scales.effort)^2;
        valid = valid + 1;
    else
        cost = cost + weights.failure;
    end
end
% Average successful runs so a batch size change does not rescale the cost.
if valid > 0, cost = cost / valid; end
detail = struct('batch', batch, 'failed', batch.successCount ~= numel(scenarios), ...
    'candidate', candidate, 'weights', weights, 'scales', scales);
end

function weights = defaults(weights)
%DEFAULTS Fill omitted objective weights without overriding user choices.
fallback = agc.opt.objectiveWeights();
names = fieldnames(fallback);
for k = 1:numel(names)
    if ~isfield(weights, names{k}), weights.(names{k}) = fallback.(names{k}); end
end
end
