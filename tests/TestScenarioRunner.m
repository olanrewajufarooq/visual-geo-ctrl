classdef TestScenarioRunner < matlab.unittest.TestCase
    methods (Test)
        function nominalRunLogsFiniteTheorySignals(testCase)
            scenario = localScenario('nominal');

            run = agc.sim.runScenario(scenario);

            testCase.verifyEqual(numel(run.t), 11);
            testCase.verifyTrue(all(isfinite(run.V), 'all'));
            testCase.verifyTrue(all(isfinite(run.wrench), 'all'));
            testCase.verifyEqual(size(run.s), [11, 6]);
            testCase.verifyTrue(all(run.Psi >= -1e-12));
            testCase.verifySize(run.estimatePi, [11, 10]);
            testCase.verifyEqual(run.estimatePi, repmat(scenario.plantPi.', 11, 1), 'AbsTol', 1e-12);
        end

        function bregmanRunKeepsEveryLoggedEstimateSPD(testCase)
            scenario = localScenario('bregman');
            scenario.controller.gammaB = 2;
            scenario.initialEstimate = agc.math.pseudoFromPi(scenario.plantPi);

            run = agc.sim.runScenario(scenario);

            testCase.verifyTrue(all(run.minPseudoEigenvalue > 0));
            testCase.verifySize(run.estimatePi, [11, 10]);
            testCase.verifyEqual(run.estimatePi(end,:).', ...
                agc.math.piFromPseudo(run.finalEstimate), 'AbsTol', 1e-12);
        end

        function payloadDropSwitchesPlantWithoutResettingNominalEstimate(testCase)
            scenario = localScenario('nominal');
            payload = struct('mass', 0.2, 'dimensions', [0.1; 0.08; 0.06], ...
                'center', [0.1; 0; -0.05]);
            loadedPi = agc.plant.compoundPi(scenario.plantPi, payload);
            scenario.duration = 0.04;
            scenario.dtControl = 0.01;
            scenario.dtAdaptation = 0.01;
            scenario.initialEstimate = loadedPi;
            scenario.payloadDrop = struct('releaseTime', 0.02, ...
                'barePi', scenario.plantPi, 'loadedPi', loadedPi, 'payload', payload);

            loadedOnly = scenario;
            loadedOnly = rmfield(loadedOnly, 'payloadDrop');
            loadedOnly.plantPi = loadedPi;
            dropRun = agc.sim.runScenario(scenario);
            loadedRun = agc.sim.runScenario(loadedOnly);

            testCase.verifyEqual(dropRun.activePlantPi(1:2,:), repmat(loadedPi.', 2, 1), 'AbsTol', 1e-12);
            testCase.verifyEqual(dropRun.activePlantPi(3:end,:), repmat(scenario.plantPi.', 3, 1), 'AbsTol', 1e-12);
            testCase.verifyEqual(dropRun.estimatePi, repmat(loadedPi.', 5, 1), 'AbsTol', 1e-12);
            testCase.verifyEqual(dropRun.H(:,:,3), loadedRun.H(:,:,3), 'AbsTol', 1e-12);
            testCase.verifyEqual(dropRun.V(3,:), loadedRun.V(3,:), 'AbsTol', 1e-12);
        end
    end
end

function scenario = localScenario(mode)
pi = [1.2; 0; 0; 0; 0.08; 0.09; 0.1; 0; 0; 0];
scenario = struct();
scenario.plantPi = pi;
scenario.initial = struct('H', [eye(3), [0.1; 0; 0]; 0, 0, 0, 1], 'V', zeros(6,1));
scenario.trajectory = @(~) struct('H', eye(4), 'V', zeros(6,1), 'Vdot', zeros(6,1));
scenario.duration = 0.1;
scenario.dtPlant = 0.01;
scenario.dtControl = 0.02;
scenario.dtAdaptation = 0.02;
scenario.plantGravity = [0; 0; -9.81];
scenario.controller = struct('mode', mode, 'coriolis', 'c2', 'KR', eye(3), ...
    'Kxi', eye(3), 'Lambda', eye(6), 'kd', 1, 'ks', 0.5, 'alpha', 0.5, ...
    'gravity', [0; 0; 9.81], 'gammaE', 0.01 * ones(10,1), 'gammaB', 0.1);
scenario.initialEstimate = pi;
end
