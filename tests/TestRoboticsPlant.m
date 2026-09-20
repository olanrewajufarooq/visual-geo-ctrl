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

        function plantUsesPaperProductsOfInertiaOrdering(testCase)
            % The paper stores [Ixy Ixz Iyz]; RST expects [Iyz Ixz Ixy].
            pi = [3.4; 0.08; -0.04; 0.02; 0.60; 0.80; 1.10; 0.05; -0.03; 0.04];
            state = struct('H', eye(4), 'V', zeros(6,1));
            wrench = [0.30; -0.40; 0.20; 1.00; -0.50; 2.00];
            plant = agc.plant.floatingBody(pi, zeros(3,1));

            acceleration = agc.plant.acceleration(plant, state, wrench);
            expected = agc.math.inertiaFromPi(pi) \ wrench;

            testCase.verifyEqual(acceleration, expected, 'AbsTol', 1e-10);
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

        function fixedPayloadMatchesHandDerivedCompoundInertia(testCase)
            barePi = [3.646; 0; 0; -0.00835; 0.04092; 0.04017; 0.06921; ...
                5.656e-5; 1.313e-5; -6.494e-5];
            payload = struct('mass', 0.75, 'dimensions', [0.12; 0.12; 0.08], ...
                'center', [0.15; 0; -0.10]);
            loadedPi = handDerivedLoadedPi(barePi, payload);
            state = struct('H', eye(4), 'V', [0.4; -0.2; 0.3; 0.5; -0.1; 0.2]);
            wrench = [0.3; -0.4; 0.2; 1.0; -0.5; 2.0];

            payloadPlant = agc.plant.floatingBody(barePi, [0; 0; -9.81], payload);
            compoundPlant = agc.plant.floatingBody(loadedPi, [0; 0; -9.81]);

            testCase.verifyEqual(agc.plant.acceleration(payloadPlant, state, wrench), ...
                agc.plant.acceleration(compoundPlant, state, wrench), 'AbsTol', 1e-10);
        end

        function compoundPiMatchesHandDerivedPayloadInertia(testCase)
            barePi = [3.646; 0; 0; -0.00835; 0.04092; 0.04017; 0.06921; ...
                5.656e-5; 1.313e-5; -6.494e-5];
            payload = struct('mass', 0.75, 'dimensions', [0.12; 0.12; 0.08], ...
                'center', [0.15; 0; -0.10]);

            testCase.verifyEqual(agc.plant.compoundPi(barePi, payload), ...
                handDerivedLoadedPi(barePi, payload), 'AbsTol', 1e-12);
        end
    end
end

function pi = handDerivedLoadedPi(barePi, payload)
%HANDDERIVEDLOADEDPI Compose the cuboid independently of production code.

m = payload.mass;
d = payload.dimensions(:);
r = payload.center(:);
payloadInertia = m / 12 * diag([d(2)^2 + d(3)^2, ...
    d(1)^2 + d(3)^2, d(1)^2 + d(2)^2]);
bareInertia = [barePi(5), barePi(8), barePi(9); ...
    barePi(8), barePi(6), barePi(10); ...
    barePi(9), barePi(10), barePi(7)];
loadedInertia = bareInertia + payloadInertia + m * ((r.' * r) * eye(3) - r * r.');
pi = [barePi(1) + m; barePi(2:4) + m * r; loadedInertia(1,1); ...
    loadedInertia(2,2); loadedInertia(3,3); loadedInertia(1,2); ...
    loadedInertia(1,3); loadedInertia(2,3)];
end
