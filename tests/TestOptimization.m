classdef TestOptimization < matlab.unittest.TestCase
    methods (Test)
        function defaultConvergenceUsesTenStalledIterationsAtOneMilliunit(testCase)
            options = agc.opt.optimizationOptions(struct());

            testCase.verifyEqual(options.functionTolerance, 1e-3);
            testCase.verifyEqual(options.maxStallIterations, 10);
            testCase.verifyEqual(options.maxIterations, 20);
        end

        function optimizerDefaultsUseSharedVelocityAndEffortWeights(testCase)
            weights = agc.opt.objectiveWeights();
            options = agc.opt.optimizationOptions(struct());

            testCase.verifyEqual(weights, struct('position', 2, 'attitude', 2, ...
                'mass', 2.5, 'cog', 2.5, 'linVel', 0.5, 'angVel', 0.5, 'inertia', 1.5, ...
                'effort', 0.01, 'failure', 1e6));
            testCase.verifyEqual(options.weights, weights);
        end

        function optimizerUsesExplicitPhysicalErrorScales(testCase)
            scales = agc.opt.objectiveScales();

            testCase.verifyEqual(scales, struct('position', 0.05, 'attitude', 0.05, ...
                'mass', 0.05, 'cog', 0.01, 'linVel', 0.20, 'angVel', 0.50, ...
                'inertia', 0.05, 'effort', 50));
            testCase.verifyEqual(agc.opt.optimizationOptions(struct()).scales, scales);
        end

        function suppliedConvergenceSettingsOverrideDefaults(testCase)
            options = agc.opt.optimizationOptions( ...
                struct('functionTolerance', 2e-4, 'maxStallIterations', 7));

            testCase.verifyEqual(options.functionTolerance, 2e-4);
            testCase.verifyEqual(options.maxStallIterations, 7);
        end

        function blankSelectorsExpandToCompleteModeAndFactorizationMatrix(testCase)
            variants = agc.opt.expandScenarioSelection('', '');

            testCase.verifyEqual(variants, { ...
                'nominal', 'c1'; 'nominal', 'c2'; ...
                'euclidean', 'c1'; 'euclidean', 'c2'; ...
                'bregman', 'c1'; 'bregman', 'c2'});
        end

        function listSelectorsFormTheirCartesianProduct(testCase)
            variants = agc.opt.expandScenarioSelection({'bregman', 'euclidean'}, 'c2');

            testCase.verifyEqual(variants, {'bregman', 'c2'; 'euclidean', 'c2'});
        end

        function gainBlocksCoverAndPartitionAdaptiveControllers(testCase)
            allIndices = agc.opt.gainBlockIndices('euclidean', 'all');
            nonadaptive = agc.opt.gainBlockIndices('euclidean', 'nonadaptive');
            adaptive = agc.opt.gainBlockIndices('euclidean', 'adaptive');

            testCase.verifyEqual(allIndices, 1:25);
            testCase.verifyEqual(nonadaptive, 1:15);
            testCase.verifyEqual(adaptive, 16:25);
            testCase.verifyEmpty(intersect(nonadaptive, adaptive));
        end

        function bregmanGridIncludesBoundsAndSeedValues(testCase)
            grid = agc.opt.bregmanGammaGrid([1e-3, 3e-4, 1e-5]);

            testCase.verifyEqual(grid(1), -5, 'AbsTol', 1e-14);
            testCase.verifyEqual(grid(end), -1, 'AbsTol', 1e-14);
            testCase.verifyTrue(any(abs(grid - log10(3e-4)) < 1e-14));
            testCase.verifyEqual(numel(grid), numel(unique(grid)));
        end

        function incumbentSelectionRetainsFeasibleLowerCostCandidate(testCase)
            incumbent = struct('candidate', [1, 2], 'cost', 4, 'failed', false, 'label', 'incumbent');
            worse = struct('candidate', [3, 4], 'cost', 5, 'failed', false, 'label', 'stage');
            failed = struct('candidate', [5, 6], 'cost', 1, 'failed', true, 'label', 'stage');
            better = struct('candidate', [7, 8], 'cost', 3, 'failed', false, 'label', 'stage');

            testCase.verifyEqual(agc.opt.bestFeasibleCandidate(incumbent, worse), incumbent);
            testCase.verifyEqual(agc.opt.bestFeasibleCandidate(incumbent, failed), incumbent);
            testCase.verifyEqual(agc.opt.bestFeasibleCandidate(incumbent, better), better);
        end

        function stagedScheduleUsesJointSearchBlockSweepAndFinalPolish(testCase)
            testCase.verifyEqual(agc.opt.gainOptimizationStages('nominal'), {'all'});
            testCase.verifyEqual(agc.opt.gainOptimizationStages('euclidean'), ...
                {'all', 'nonadaptive', 'adaptive', 'all'});
            testCase.verifyEqual(agc.opt.gainOptimizationStages('bregman'), ...
                {'all', 'nonadaptive', 'adaptive', 'all'});
        end
    end
end
