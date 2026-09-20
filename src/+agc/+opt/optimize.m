function result = optimize(baseScenarios, decode, lowerBound, upperBound, options)
%OPTIMIZE Run particleswarm against the explicit scenario objective.

%% Optimizer defaults and candidate bounds

if nargin < 5, options = struct(); end
options = agc.opt.optimizationOptions(options);
lowerBound = lowerBound(:).'; upperBound = upperBound(:).';
if numel(lowerBound) ~= numel(upperBound) || any(lowerBound >= upperBound)
    error('agc:opt:optimize:Bounds', 'Bounds must have equal widths with lower < upper.');
end
%% Particle-level parallel objective

% Parallelize particles (candidate evaluations), not scenarios inside each
% particle, to avoid nested parfor/particleswarm worker pools.
objectiveHandle = @(x) agc.opt.objective(x, baseScenarios, decode, options.weights, false);
solverOptions = optimoptions('particleswarm', 'Display', 'iter', ...
    'SwarmSize', options.swarmSize, 'MaxIterations', options.maxIterations, ...
    'FunctionTolerance', options.functionTolerance, ...
    'MaxStallIterations', options.maxStallIterations, ...
    'UseParallel', logical(options.parallel));
if ~isempty(options.initialPoints)
    initialPoints = options.initialPoints;
    if isvector(initialPoints), initialPoints = initialPoints(:).'; end
    if size(initialPoints, 2) ~= numel(lowerBound)
        error('agc:opt:optimize:InitialPointsWidth', ...
            'Each initial point must have %d coordinates.', numel(lowerBound));
    end
    % Historical optimized entries can predate tightened bounds. Project only
    % the seed point to the valid search box; the manual seed is unchanged.
    initialPoints = min(max(initialPoints, lowerBound), upperBound);
    solverOptions.InitialPoints = initialPoints;
end
[candidate, cost] = particleswarm(objectiveHandle, numel(lowerBound), lowerBound, upperBound, solverOptions);

%% Re-evaluate the winner with the requested batch parallelism

[~, detail] = agc.opt.objective(candidate, baseScenarios, decode, options.weights, logical(options.parallel));
result = struct('candidate', candidate, 'cost', cost, 'detail', detail, 'options', options);
end
