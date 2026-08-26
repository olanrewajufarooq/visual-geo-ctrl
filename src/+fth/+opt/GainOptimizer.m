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

                fullDimension = problem.dimension();
                activeMask = opts.optimizationMask;
                if isempty(activeMask), activeMask = true(1, fullDimension); end
                activeMask = logical(activeMask(:).');
                if numel(activeMask) ~= fullDimension || ~any(activeMask)
                    error('fth:GainOptimizer:InvalidOptimizationMask', ...
                        'optimizationMask must match the problem dimension and select variables.');
                end
                fixedVector = problem.BaselineVector;
                if ~isempty(opts.initialVector)
                    fixedVector = double(opts.initialVector(:).');
                    if numel(fixedVector) ~= fullDimension
                        error('fth:GainOptimizer:InvalidInitialVector', ...
                            'initialVector must match the problem dimension.');
                    end
                end

                iterationOffset = 0;
                elapsedBefore = 0;
                history = zeros(0, 3);
                improvementHistory = fth.opt.GainOptimizationIO.emptyGainHistory(18 + ...
                    fth.opt.GainOptimizationUtils.gammaWidth(scenario.adaptation));
                initialFull = fth.opt.GainOptimizationUtils.initialSwarm(problem, opts.swarmSize);
                initialFull(:, ~activeMask) = repmat(fixedVector(~activeMask), size(initialFull, 1), 1);
                initial = initialFull(:, activeMask);
                if found && ~clearCache && isfield(cached, 'lastSwarm') && ...
                        isfield(cached, 'completedIterations') && ...
                        size(cached.lastSwarm, 2) == nnz(activeMask)
                    initial = cached.lastSwarm;
                    iterationOffset = cached.completedIterations;
                    if isfield(cached, 'history'), history = cached.history; end
                    if isfield(cached, 'improvementHistory')
                        improvementHistory = cached.improvementHistory;
                    end
                    if isfield(cached, 'elapsedSeconds'), elapsedBefore = cached.elapsedSeconds; end
                    fprintf('[%s] Resuming after iteration %d from %s.\n', ...
                        scenario.id, iterationOffset, cacheFolder);
                end
                reportFolder = fullfile(root, timestamp, scenario.id);
                if ~exist(cacheFolder, 'dir'), mkdir(cacheFolder); end
                if ~exist(reportFolder, 'dir'), mkdir(reportFolder); end

                lastSwarm = initial;
                bestXCheckpoint = fixedVector;
                bestCostCheckpoint = inf;
                if found && ~clearCache && isfield(cached, 'bestCost') && isfield(cached, 'bestX')
                    bestCostCheckpoint = cached.bestCost;
                    bestXCheckpoint = cached.bestX;
                end
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
                    @(x) problem.evaluate(expandVector(x)), nnz(activeMask), ...
                    problem.LowerBound(activeMask), problem.UpperBound(activeMask), psOpts);
                bestX = expandVector(bestX);
                elapsed = elapsedBefore + toc(startTimer);
                [~, bestBreakdown] = problem.evaluate(bestX);
                bestXCheckpoint = bestX;
                bestCostCheckpoint = bestCost;
                completedIterations = max(completedIterations, iterationOffset + output.iterations);
                if isempty(improvementHistory) || bestCost < improvementHistory.best_cost(end)
                    improvementHistory(end+1, :) = fth.opt.GainOptimizationIO.gainTable( ...
                        completedIterations, bestCost, elapsed, bestX, 'final'); %#ok<AGROW>
                end
                saveCheckpoint(true);
                completed = true;

                save(fullfile(reportFolder, 'optimizer_state.mat'), 'bestX', 'bestCost', ...
                    'exitflag', 'output', 'history', 'baseline', 'baselineBreakdown', ...
                    'bestBreakdown', 'opts', 'scenario', 'runSignature', 'lastSwarm', ...
                    'completedIterations', 'elapsed', 'completed', 'improvementHistory');
                fth.opt.GainOptimizationIO.writeConvergence(reportFolder, history);
                fth.opt.GainOptimizationIO.writeImprovements(reportFolder, improvementHistory);
                fth.opt.GainOptimizationIO.writeBaselineAndBest(reportFolder, ...
                    problem.BaselineVector, baseline, bestX, bestCost);
                fth.opt.GainOptimizationIO.writeMetadata(reportFolder, scenario, opts, ...
                    runSignature, exitflag, output, completed, elapsed);
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
                    candidateCost = fth.opt.GainOptimizationUtils.bestParticleValue(optimValues);
                    previousCost = bestCostCheckpoint;
                    bestCostCheckpoint = candidateCost;
                    if isfield(optimValues, 'bestx')
                        bestXCheckpoint = expandVector(optimValues.bestx);
                    end
                    if isfield(optimValues, 'swarm')
                        lastSwarm = optimValues.swarm;
                    end
                    completedIterations = iterationOffset + optimValues.iteration;
                    elapsedNow = elapsedBefore + toc(startTimer);
                    history(end+1, :) = [completedIterations, bestCostCheckpoint, elapsedNow];
                    if isfinite(candidateCost) && (isempty(improvementHistory) || ...
                            ~isfinite(previousCost) || candidateCost < previousCost)
                        improvementHistory(end+1, :) = fth.opt.GainOptimizationIO.gainTable( ...
                            completedIterations, candidateCost, elapsedNow, bestXCheckpoint, ...
                            'improvement'); %#ok<AGROW>
                    end
                    saveCheckpoint(false);
                end
            end

            function fullX = expandVector(activeX)
                fullX = fixedVector;
                fullX(activeMask) = double(activeX(:).');
            end

            function saveCheckpoint(isComplete)
                bestX = bestXCheckpoint;
                bestCost = bestCostCheckpoint;
                elapsedSeconds = elapsedBefore + toc(startTimer);
                completed = isComplete;
                save(fullfile(cacheFolder, 'optimizer_state.mat'), 'bestX', 'bestCost', ...
                    'history', 'baseline', 'baselineBreakdown', 'opts', 'scenario', ...
                    'runSignature', 'lastSwarm', 'completedIterations', ...
                    'elapsedSeconds', 'completed', 'improvementHistory');
            end
        end
    end
end
