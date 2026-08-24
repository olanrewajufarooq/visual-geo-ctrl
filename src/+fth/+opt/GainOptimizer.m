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
            root = fullfile('results', 'tuning', timestamp);
            if ~exist(root, 'dir'), mkdir(root); end

            for i = 1:numel(scenarios)
                scenario = scenarios(i);
                fprintf('\n[%d/%d] %s\n', i, numel(scenarios), scenario.label);
                problem = fth.opt.GainEvaluationProblem(scenario.adaptation, ...
                    {fth.opt.GainOptimizationScenario.build(scenario, opts.duration)}, ...
                    struct('bounds', opts.bounds));
                [baseline, baselineBreakdown] = problem.evaluateBaseline();
                if any(problem.BaselineFailures)
                    warning('fth:GainOptimizer:BaselineFallback', ...
                        '%s baseline diverged; fallback normalizers will be used.', scenario.id);
                end

                initial = fth.opt.GainOptimizationUtils.initialSwarm(problem, opts.swarmSize);
                history = zeros(0, 3);
                startTimer = tic;
                psOpts = optimoptions('particleswarm', ...
                    'SwarmSize', opts.swarmSize, 'MaxIterations', opts.maxIterations, ...
                    'UseParallel', opts.useParallel, 'UseVectorized', false, ...
                    'InitialSwarmMatrix', initial, 'Display', 'iter', ...
                    'FunctionTolerance', opts.functionTolerance, ...
                    'MaxStallIterations', opts.maxStallIterations, ...
                    'OutputFcn', @recordHistory);
                [bestX, bestCost, exitflag, output] = particleswarm( ...
                    @(x) problem.evaluate(x), problem.dimension(), ...
                    problem.LowerBound, problem.UpperBound, psOpts);
                elapsed = toc(startTimer);
                [~, bestBreakdown] = problem.evaluate(bestX);

                folder = fullfile(root, scenario.id);
                if ~exist(folder, 'dir'), mkdir(folder); end
                save(fullfile(folder, 'optimizer_state.mat'), 'bestX', 'bestCost', ...
                    'exitflag', 'output', 'history', 'baseline', 'baselineBreakdown', ...
                    'bestBreakdown', 'opts', 'scenario');
                fth.opt.GainOptimizationIO.writeConvergence(folder, history);
                writematrix([problem.BaselineVector; bestX], ...
                    fullfile(folder, 'baseline_and_best.csv'));
                fth.opt.GainOptimizationIO.writeSummary(folder, scenario, bestX, ...
                    bestCost, elapsed, bestBreakdown);
                fth.opt.GainOptimizationIO.writeScript(folder, scenario, bestX, bestCost);
                fth.opt.GainOptimizationIO.plotConvergence(folder, history, scenario.id);
                fth.opt.GainOptimizationIO.printBestGains(scenario, bestX, bestCost);
                fprintf('[%s] Best cost %.6g after %.1f s.\n', scenario.id, bestCost, elapsed);
            end

            function stop = recordHistory(optimValues, state)
                stop = false;
                if strcmp(state, 'iter')
                    history(end+1, :) = [optimValues.iteration, ...
                        fth.opt.GainOptimizationUtils.bestParticleValue(optimValues), toc(startTimer)];
                end
            end
        end
    end
end
