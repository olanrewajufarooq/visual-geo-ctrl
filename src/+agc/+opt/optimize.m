function result = optimize(baseScenarios, decode, lowerBound, upperBound, options)
%OPTIMIZE Run particleswarm against the explicit scenario objective.

%% Optimizer defaults and candidate bounds

if nargin < 5, options = struct(); end
if ~isfield(options, 'weights'), options.weights = struct(); end
if ~isfield(options, 'swarmSize'), options.swarmSize = 30; end
if ~isfield(options, 'maxIterations'), options.maxIterations = 50; end
if ~isfield(options, 'parallel'), options.parallel = true; end
lowerBound = lowerBound(:).'; upperBound = upperBound(:).';
if numel(lowerBound) ~= numel(upperBound) || any(lowerBound >= upperBound)
    error('agc:opt:optimize:Bounds', 'Bounds must have equal widths with lower < upper.');
end
%% Particle-level parallel objective

% Parallelize particles (candidate evaluations), not scenarios inside each
% particle, to avoid nested parfor/particleswarm worker pools.
objectiveHandle = @(x) agc.opt.objective(x, baseScenarios, decode, options.weights, false);
solverOptions = optimoptions('particleswarm', 'Display', 'iter', ...
    'SwarmSize', options.swarmSize, 'MaxIterations', options.maxIterations, 'UseParallel', logical(options.parallel));
[candidate, cost] = particleswarm(objectiveHandle, numel(lowerBound), lowerBound, upperBound, solverOptions);

%% Re-evaluate the winner with the requested batch parallelism

[~, detail] = agc.opt.objective(candidate, baseScenarios, decode, options.weights, logical(options.parallel));
result = struct('candidate', candidate, 'cost', cost, 'detail', detail, 'options', options);
end
