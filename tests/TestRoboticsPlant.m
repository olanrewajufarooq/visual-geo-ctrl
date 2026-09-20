classdef TestRoboticsPlant < matlab.unittest.TestCase
    methods (Test)
        function gravityProducesBodyAccelerationAtIdentity(testCase)
            pi = [2; 0; 0; 0; 0.1; 0.12; 0.14; 0; 0; 0];
            plant = agc.plant.floatingBody(pi, [0; 0; -9.81]);
            state = struct('H', eye(4), 'V', zeros(6,1));

            acceleration = agc.plant.acceleration(plant, state, zeros(6,1));

            testCase.verifyEqual(acceleration, [zeros(5,1); -9.81], 'AbsTol', 1e-10);
        end

        function bodyWrenchCancelsGravityAtHover(testCase)
            pi = [2; 0; 0; 0; 0.1; 0.12; 0.14; 0; 0; 0];
            gravity = [0; 0; -9.81];
            plant = agc.plant.floatingBody(pi, gravity);
            state = struct('H', eye(4), 'V', zeros(6,1));
            hoverWrench = [zeros(3,1); -pi(1) * gravity];

            acceleration = agc.plant.acceleration(plant, state, hoverWrench);

            testCase.verifyEqual(acceleration, zeros(6,1), 'AbsTol', 1e-10);
        end

        function paperGravityConventionCancelsRoboticsGravityAtHover(testCase)
            pi = [2; 0; 0; 0; 0.1; 0.12; 0.14; 0; 0; 0];
            state = struct('H', eye(4), 'V', zeros(6,1));
            desired = struct('H', eye(4), 'V', zeros(6,1), 'Vdot', zeros(6,1));
            cfg = struct('mode', 'nominal', 'coriolis', 'c2', 'KR', eye(3), ...
                'Kxi', eye(3), 'Lambda', eye(6), 'kd', 1, 'ks', 1, ...
                'alpha', 0.5, 'gravity', [0; 0; 9.81]);
            plant = agc.plant.floatingBody(pi, [0; 0; -9.81]);

            wrench = agc.paper.controller(state, desired, cfg, pi, []);
            acceleration = agc.plant.acceleration(plant, state, wrench);

            testCase.verifyEqual(acceleration, zeros(6,1), 'AbsTol', 1e-10);
        end

        function propagatedQuaternionRemainsUnitLength(testCase)
            pi = [1; 0; 0; 0; 0.1; 0.1; 0.1; 0; 0; 0];
            plant = agc.plant.floatingBody(pi, zeros(3,1));
            state = struct('H', eye(4), 'V', [0; 0; 2; 1; 0; 0]);

            next = agc.plant.propagate(plant, state, zeros(6,1), 0.05);

            testCase.verifyEqual(next.H(1:3,1:3).' * next.H(1:3,1:3), eye(3), 'AbsTol', 1e-12);
            testCase.verifyGreaterThan(next.H(1,4), 0);
        end
    end
end
