classdef TestWorkflow < matlab.unittest.TestCase
    methods (Test)
        function batchUsesSingleScenarioRunnerContract(testCase)
            scenario = makeScenario();

            batch = agc.batch.runBatch({scenario, scenario}, struct('parallel', false));

            testCase.verifyEqual(batch.successCount, 2);
            testCase.verifyEqual(numel(batch.metrics), 2);
            testCase.verifyGreaterThanOrEqual(batch.metrics{1}.positionRMSE, 0);
        end

        function replaySamplerReturnsPaperDesiredState(testCase)
            sampler = agc.sim.replayTrajectory(fullfile('trajectories', 'processed', 'lemniscate_01_auto.mat'));

            desired = sampler(0.1);

            testCase.verifySize(desired.H, [4, 4]);
            testCase.verifySize(desired.V, [6, 1]);
            testCase.verifySize(desired.Vdot, [6, 1]);
            testCase.verifyEqual(desired.H(4,:), [0, 0, 0, 1]);
        end

        function objectiveEvaluatesDecodedScenarioWithExplicitWeights(testCase)
            scenario = makeScenario();
            weights = struct('position', 1, 'attitude', 2, 'effort', 0, 'failure', 1e6);

            [cost, detail] = agc.opt.objective(0, {scenario}, @(~, base) base, weights, false);

            testCase.verifyFalse(detail.failed);
            testCase.verifyGreaterThanOrEqual(cost, 0);
            testCase.verifyEqual(numel(detail.batch.metrics), 1);
            testCase.verifyFalse(detail.batch.parallel);
        end

        function objectiveCanWeightNormalizedParameterEstimationRmse(testCase)
            scenario = makeScenario();
            scenario.initialEstimate(1) = 1.1 * scenario.plantPi(1);
            weights = struct('position', 0, 'attitude', 0, 'effort', 0, ...
                'estimation', 1, 'failure', 1e6);

            [cost, detail] = agc.opt.objective(0, {scenario}, @(~, base) base, weights, false);

            testCase.verifyGreaterThan(detail.batch.metrics{1}.parameterEstimationRMSE, 0);
            testCase.verifyEqual(cost, detail.batch.metrics{1}.parameterEstimationRMSE, 'AbsTol', 1e-12);
        end

        function defaultScenarioUsesPreservedReplayArtifact(testCase)
            scenario = agc.sim.defaultScenario('lemniscate_01_auto', 'nominal', 'c2', 0.1);

            desired = scenario.trajectory(0);

            testCase.verifyEqual(scenario.duration, 0.1);
            testCase.verifySize(desired.H, [4, 4]);
            testCase.verifyTrue(agc.math.isSPD(agc.math.pseudoFromPi(scenario.plantPi)));
        end

        function defaultScenarioSeparatesPaperAndPlantGravityConventions(testCase)
            scenario = agc.sim.defaultScenario('lemniscate_01_auto', 'nominal', 'c2', 0.1);

            testCase.verifyEqual(scenario.controller.gravity, [0; 0; 9.81]);
            testCase.verifyEqual(scenario.plantGravity, [0; 0; -9.81]);
        end

        function defaultPayloadDropStartsEveryModeAtExactLoadedParameters(testCase)
            nominal = agc.sim.defaultScenario('lemniscate_01_auto', 'nominal', 'c1', 11);
            euclidean = agc.sim.defaultScenario('lemniscate_01_auto', 'euclidean', 'c1', 11);
            bregman = agc.sim.defaultScenario('lemniscate_01_auto', 'bregman', 'c1', 11);

            testCase.verifyEqual(nominal.payloadDrop.releaseTime, 10);
            testCase.verifyEqual(nominal.initialEstimate, nominal.payloadDrop.loadedPi, 'AbsTol', 1e-12);
            testCase.verifyEqual(euclidean.initialEstimate, nominal.payloadDrop.loadedPi, 'AbsTol', 1e-12);
            testCase.verifyEqual(agc.math.piFromPseudo(bregman.initialEstimate), ...
                nominal.payloadDrop.loadedPi, 'AbsTol', 1e-12);
            testCase.verifyEqual(nominal.payloadDrop.barePi, nominal.plantPi, 'AbsTol', 1e-12);
            testCase.verifyTrue(agc.math.isSPD(agc.math.pseudoFromPi(nominal.payloadDrop.loadedPi)));
        end

        function defaultScenarioUsesOptimizedDiagonalGainEntry(testCase)
            scenario = agc.sim.defaultScenario('lemniscate_01_auto', 'euclidean', 'c2', 0.1);
            gains = optimized_gains('euclidean', 'c2');

            testCase.verifyEqual(scenario.controller.KR, diag(gains.KRdiag));
            testCase.verifyEqual(scenario.controller.Kxi, diag(gains.Kxidiag));
            testCase.verifyEqual(scenario.controller.Lambda, diag(gains.LambdaDiag));
            testCase.verifyEqual(scenario.controller.gammaE, gains.gammaE);
            testCase.verifyEqual(scenario.controller.gammaB, gains.gammaB);
            testCase.verifyFalse(isfield(scenario.controller, 'Gamma'));
        end

        function defaultScenarioRunsThroughRoboticsBackend(testCase)
            scenario = agc.sim.defaultScenario('lemniscate_01_auto', 'nominal', 'c1', 0.02);
            scenario.payloadDrop.releaseTime = 0.01;

            run = agc.sim.runScenario(scenario);

            testCase.verifyTrue(all(isfinite(run.V), 'all'));
            testCase.verifyTrue(all(isfinite(run.wrench), 'all'));
        end
    end
end

function scenario = makeScenario()
pi = [1; 0; 0; 0; 0.1; 0.1; 0.1; 0; 0; 0];
scenario = struct('plantPi', pi, 'initial', struct('H', eye(4), 'V', zeros(6,1)), ...
    'trajectory', @(~) struct('H', eye(4), 'V', zeros(6,1), 'Vdot', zeros(6,1)), ...
    'duration', 0.02, 'dtPlant', 0.01, 'dtControl', 0.01, 'dtAdaptation', 0.01, ...
    'plantGravity', [0; 0; -9.81], ...
    'controller', struct('mode', 'nominal', 'coriolis', 'c1', 'KR', eye(3), ...
        'Kxi', eye(3), 'Lambda', eye(6), 'kd', 1, 'ks', 0.5, 'alpha', 0.5, ...
        'gravity', [0; 0; 9.81], 'gammaE', ones(10,1), 'gammaB', 1), ...
    'initialEstimate', pi);
end
