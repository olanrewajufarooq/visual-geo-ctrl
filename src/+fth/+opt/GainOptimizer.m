classdef GainOptimizer
    %GAINOPTIMIZER Orchestrate independent gain optimizations.

    methods (Static)
        function run(overrides)
            opts = fth.opt.GainOptimizationUtils.defaults();
            if nargin >= 1 && ~isempty(overrides)
                if ~isstruct(overrides)
                    error('fth:GainOptimizer:InvalidOptions', 'Options must be a struct.');
                end
                names = fieldnames(overrides);
                for i = 1:numel(names), opts.(names{i}) = overrides.(names{i}); end
            end

            catalog = fth.opt.GainOptimizationScenario.catalog();
            scenarios = fth.opt.GainOptimizationScenario.select(catalog, opts.scenarios);
            fth.opt.GainOptimizationUtils.validateEnvironment(opts.useParallel);
            rng(opts.randomSeed, 'twister');

            timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
            root = opts.outputRoot;
            if ~exist(root, 'dir'), mkdir(root); end
            if ~exist(opts.cacheRoot, 'dir'), mkdir(opts.cacheRoot); end

            for i = 1:numel(scenarios)
                scenario = scenarios(i);
                fprintf('\n[%d/%d] %s\n', i, numel(scenarios), scenario.label);
                clearCache = opts.clearCache;
                if isfield(scenario, 'clearCache') && ~isempty(scenario.clearCache)
                    clearCache = logical(scenario.clearCache);
                end
                runSignature = fth.opt.GainOptimizationIO.makeSignature(opts, scenario);
                [cached, folder, found] = fth.opt.GainOptimizationIO.findState( ...
                    opts.cacheRoot, scenario.id, runSignature);
                cacheFolder = folder;
                if clearCache
                    fth.opt.GainOptimizationIO.clearState(opts.cacheRoot, scenario.id);
                    cached = struct();
                    found = false;
                end
                if found && ~clearCache && isfield(cached, 'completed') && cached.completed
                    fprintf('[%s] Reusing completed result from %s.\n', scenario.id, cacheFolder);
                    fth.opt.GainOptimizationIO.printBestGains(scenario, cached.bestX, cached.bestCost);
                    continue;
                end

                problem = fth.opt.GainEvaluationProblem(scenario.adaptation, ...
                    {fth.opt.GainOptimizationScenario.build(scenario, opts.duration)}, ...
                    struct('bounds', opts.bounds));
                [baseline, baselineBreakdown] = problem.evaluateBaseline();
                if any(problem.BaselineFailures)
                    warning('fth:GainOptimizer:BaselineFallback', ...
                        '%s baseline diverged; fallback normalizers will be used.', scenario.id);
                end

                iterationOffset = 0;
                elapsedBefore = 0;
                history = zeros(0, 3);
                initial = fth.opt.GainOptimizationUtils.initialSwarm(problem, opts.swarmSize);
                if found && ~clearCache && isfield(cached, 'lastSwarm') && ...
                        isfield(cached, 'completedIterations') && ...
                        size(cached.lastSwarm, 2) == problem.dimension()
                    initial = cached.lastSwarm;
                    iterationOffset = cached.completedIterations;
                    if isfield(cached, 'history'), history = cached.history; end
                    if isfield(cached, 'elapsedSeconds'), elapsedBefore = cached.elapsedSeconds; end
                    fprintf('[%s] Resuming after iteration %d from %s.\n', ...
                        scenario.id, iterationOffset, cacheFolder);
                end
                reportFolder = fullfile(root, timestamp, scenario.id);
                if ~exist(cacheFolder, 'dir'), mkdir(cacheFolder); end
                if ~exist(reportFolder, 'dir'), mkdir(reportFolder); end

                lastSwarm = initial;
                bestXCheckpoint = problem.BaselineVector;
                bestCostCheckpoint = inf;
                completedIterations = iterationOffset;
                output = struct();
                exitflag = NaN;
                startTimer = tic;
                remainingIterations = max(1, opts.maxIterations - iterationOffset);
                psOpts = optimoptions('particleswarm', ...
                    'SwarmSize', size(initial, 1), 'MaxIterations', remainingIterations, ...
                    'UseParallel', opts.useParallel, 'UseVectorized', false, ...
                    'InitialSwarmMatrix', initial, 'Display', 'iter', ...
                    'FunctionTolerance', opts.functionTolerance, ...
                    'MaxStallIterations', opts.maxStallIterations, ...
                    'OutputFcn', @recordHistory);
                [bestX, bestCost, exitflag, output] = particleswarm( ...
                    @(x) problem.evaluate(x), problem.dimension(), ...
                    problem.LowerBound, problem.UpperBound, psOpts);
                elapsed = elapsedBefore + toc(startTimer);
                [~, bestBreakdown] = problem.evaluate(bestX);
                bestXCheckpoint = bestX;
                bestCostCheckpoint = bestCost;
                completedIterations = max(completedIterations, iterationOffset + output.iterations);
                saveCheckpoint(true);
                completed = true;

                save(fullfile(reportFolder, 'optimizer_state.mat'), 'bestX', 'bestCost', ...
                    'exitflag', 'output', 'history', 'baseline', 'baselineBreakdown', ...
                    'bestBreakdown', 'opts', 'scenario', 'runSignature', 'lastSwarm', ...
                    'completedIterations', 'elapsed', 'completed');
                fth.opt.GainOptimizationIO.writeConvergence(reportFolder, history);
                writematrix([problem.BaselineVector; bestX], ...
                    fullfile(reportFolder, 'baseline_and_best.csv'));
                fth.opt.GainOptimizationIO.writeSummary(reportFolder, scenario, bestX, ...
                    bestCost, elapsed, bestBreakdown);
                fth.opt.GainOptimizationIO.writeScript(reportFolder, scenario, bestX, bestCost);
                fth.opt.GainOptimizationIO.plotConvergence(reportFolder, history, scenario.id);
                fth.opt.GainOptimizationIO.printBestGains(scenario, bestX, bestCost);
                fprintf('[%s] Best cost %.6g after %.1f s.\n', scenario.id, bestCost, elapsed);
            end

            function stop = recordHistory(optimValues, state)
                stop = false;
                if strcmp(state, 'iter')
                    bestCostCheckpoint = fth.opt.GainOptimizationUtils.bestParticleValue(optimValues);
                    if isfield(optimValues, 'bestx')
                        bestXCheckpoint = optimValues.bestx;
                    end
                    if isfield(optimValues, 'swarm')
                        lastSwarm = optimValues.swarm;
                    end
                    completedIterations = iterationOffset + optimValues.iteration;
                    history(end+1, :) = [completedIterations, bestCostCheckpoint, ...
                        elapsedBefore + toc(startTimer)];
                    saveCheckpoint(false);
                end
            end

            function saveCheckpoint(isComplete)
                bestX = bestXCheckpoint;
                bestCost = bestCostCheckpoint;
                elapsedSeconds = elapsedBefore + toc(startTimer);
                completed = isComplete;
                save(fullfile(cacheFolder, 'optimizer_state.mat'), 'bestX', 'bestCost', ...
                    'history', 'baseline', 'baselineBreakdown', 'opts', 'scenario', ...
                    'runSignature', 'lastSwarm', 'completedIterations', ...
                    'elapsedSeconds', 'completed');
            end
        end
    end
end
