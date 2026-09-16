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
        end

        function bregmanRunKeepsEveryLoggedEstimateSPD(testCase)
            scenario = localScenario('bregman');
            scenario.controller.gammaB = 2;
            scenario.initialEstimate = agc.math.pseudoFromPi(scenario.plantPi);

            run = agc.sim.runScenario(scenario);

            testCase.verifyTrue(all(run.minPseudoEigenvalue > 0));
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
    'gravity', [0; 0; 9.81], 'Gamma', 0.01 * eye(10), 'gammaB', 0.1);
scenario.initialEstimate = pi;
end
