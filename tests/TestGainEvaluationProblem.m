classdef TestGainEvaluationProblem < matlab.unittest.TestCase
    methods (Test)
        function testBregmanUsesScalarGamma(testCase)
            problem = fth.opt.GainEvaluationProblem('bregman', ...
                {fth.sim.Config()}, struct('normalizers', testCase.normalizers()));
            testCase.verifyEqual(problem.dimension(), 19);
            [~, decoded] = problem.decode(problem.BaselineVector);
            testCase.verifySize(decoded.Gamma, [1 1]);
            testCase.verifyEqual(decoded.Gamma, 1e-3, 'AbsTol', 1e-12);
        end

        function testBaselineUsesStableReplayCoupling(testCase)
            problem = fth.opt.GainEvaluationProblem('bregman', ...
                {fth.sim.Config()}, struct('normalizers', testCase.normalizers()));
            testCase.verifyEqual(problem.BaselineVector(13:18), ...
                [0.5 0.5 0.5 0.2 0.2 0.2], 'AbsTol', 1e-12);
        end

        function testEuclideanUsesTenElementGamma(testCase)
            problem = fth.opt.GainEvaluationProblem('euclidean', ...
                {fth.sim.Config()}, struct('normalizers', testCase.normalizers()));
            testCase.verifyEqual(problem.dimension(), 28);
            cfg = fth.sim.Config();
            [cfg, decoded] = problem.decode(problem.BaselineVector, cfg);
            testCase.verifySize(decoded.Gamma, [10 1]);
            testCase.verifyEqual(decoded.Gamma, 1e-3 * ones(10, 1), 'AbsTol', 1e-12);
            cfg.setAdaptation('euclidean');
            cfg.done();
            testCase.verifySize(cfg.controller.Gamma, [10 1]);
        end

        function testBoundsHaveExpectedShapes(testCase)
            b = fth.opt.GainEvaluationProblem('bregman', {fth.sim.Config()}, ...
                struct('normalizers', testCase.normalizers()));
            e = fth.opt.GainEvaluationProblem('euclidean', {fth.sim.Config()}, ...
                struct('normalizers', testCase.normalizers()));
            testCase.verifySize(b.LowerBound, [1 19]);
            testCase.verifySize(e.LowerBound, [1 28]);
            testCase.verifyLessThanOrEqual(b.LowerBound, b.UpperBound);
            testCase.verifyLessThanOrEqual(e.LowerBound, e.UpperBound);
        end

        function testCustomBoundsAreApplied(testCase)
            bounds = struct('Kp', [0.01, 12], 'Kd', [0.02, 8], ...
                'lambda', [0, 25], 'gammaLog10', [-7, -2]);
            problem = fth.opt.GainEvaluationProblem('bregman', {fth.sim.Config()}, ...
                struct('normalizers', testCase.normalizers(), 'bounds', bounds));
            testCase.verifyEqual(problem.LowerBound([1, 7, 13, 19]), [0.01, 0.02, 0, -7]);
            testCase.verifyEqual(problem.UpperBound([1, 7, 13, 19]), [12, 8, 25, -2]);
        end

        function testParticleSwarmHistoryUsesBestObjectiveField(testCase)
            optimValues = struct('iteration', 4, 'bestfval', 2.75, ...
                'swarmfvals', [3.1 2.75 4.0]);
            testCase.verifyEqual(fth.opt.GainOptimizationUtils.bestParticleValue(optimValues), ...
                2.75, 'AbsTol', 1e-12);
        end

        function testPayloadScenariosUseMidVehiclePayloadInitialization(testCase)
            scenarios = fth.opt.GainOptimizationScenario.catalog();
            payload = [scenarios.withPayload];

            testCase.verifyEqual({scenarios(payload).paramInit}, ...
                repmat({'mid-vehicle-payload'}, 1, nnz(payload)));
            testCase.verifyEqual({scenarios(~payload).paramInit}, ...
                repmat({'vehicle-slight-dev'}, 1, nnz(~payload)));
        end

    end

    methods (Access = private)
        function normalizers = normalizers(~)
            normalizers = struct('position', 1, 'orientation', 1, ...
                'maxPosition', 1, 'forceRms', 1, 'torqueRms', 1, 'effort', 1);
        end
    end
end
