function batch = runBatch(scenarios, options)
%RUNBATCH Execute independent scenarios, using parallel workers by default.

%% Batch options and output allocation

if nargin < 2 || isempty(options), options = struct(); end
if ~isfield(options, 'parallel'), options.parallel = true; end
validateattributes(options.parallel, {'logical', 'numeric'}, {'scalar'});
n = numel(scenarios);
runs = cell(n,1); metrics = cell(n,1); failures = cell(n,1);
useParallel = logical(options.parallel) && ~isempty(ver('parallel'));

%% Independent simulation execution

% Each scenario owns its state, estimate, and replay log, so no worker
% communicates with another worker during a batch execution.
if useParallel
    parfor k = 1:n
        [runs{k}, metrics{k}, failures{k}] = executeOne(scenarios{k});
    end
else
    for k = 1:n
        [runs{k}, metrics{k}, failures{k}] = executeOne(scenarios{k});
    end
end
batch = struct('runs', {runs}, 'metrics', {metrics}, 'failures', {failures}, ...
    'parallel', useParallel, 'successCount', sum(cellfun(@isempty, failures)));
end

function [run, metric, failure] = executeOne(scenario)
%EXECUTEONE Convert an individual scenario failure into batch diagnostics.
try
    run = agc.sim.runScenario(scenario);
    metric = agc.sim.metrics(run);
    failure = [];
catch exception
    run = [];
    metric = [];
    failure = struct('identifier', exception.identifier, 'message', exception.message);
end
end
