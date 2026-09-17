classdef TestOptimization < matlab.unittest.TestCase
    methods (Test)
        function defaultConvergenceUsesTenStalledIterationsAtOneMilliunit(testCase)
            options = agc.opt.optimizationOptions(struct());

            testCase.verifyEqual(options.functionTolerance, 1e-3);
            testCase.verifyEqual(options.maxStallIterations, 10);
            testCase.verifyEqual(options.maxIterations, 20);
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
    end
end
