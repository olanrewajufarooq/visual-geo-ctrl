classdef TestPaperCore < matlab.unittest.TestCase
    %TESTPAPERCORE Behavioural tests for the paper-aligned SE(3) core.

    methods (Test)
        function c1AndC2AgreeOnBodyVelocity(testCase)
            pi = [2.4; 0.12; -0.07; 0.04; 0.22; 0.31; 0.38; 0.01; -0.02; 0.03];
            I6 = agc.math.inertiaFromPi(pi);
            V = [0.7; -0.3; 0.2; 1.1; -0.4; 0.6];

            c1 = agc.paper.coriolis('c1', V, I6, V);
            c2 = agc.paper.coriolis('c2', V, I6, V);

            testCase.verifyEqual(c1, c2, 'AbsTol', 1e-12);
        end

        function c1UsesLeviCivitaFactorization(testCase)
            pi = [1.9; 0.08; 0.03; -0.05; 0.19; 0.27; 0.33; 0.02; 0.01; -0.015];
            I6 = agc.math.inertiaFromPi(pi);
            V = [0.4; -0.2; 0.5; 0.7; 0.1; -0.3];
            U = [-0.6; 0.8; 0.1; -0.2; 0.9; 0.3];
            adV = agc.math.adTwist(V);
            adU = agc.math.adTwist(U);
            expected = 0.5 * (I6 * adV * U - adU.' * (I6 * V) - adV.' * (I6 * U));

            actual = agc.paper.coriolis('c1', V, I6, U);

            testCase.verifyEqual(actual, expected, 'AbsTol', 1e-12);
        end

        function c2UsesAlternativeCoadjointFactorization(testCase)
            pi = [1.9; 0.08; 0.03; -0.05; 0.19; 0.27; 0.33; 0.02; 0.01; -0.015];
            I6 = agc.math.inertiaFromPi(pi);
            V = [0.4; -0.2; 0.5; 0.7; 0.1; -0.3];
            U = [-0.6; 0.8; 0.1; -0.2; 0.9; 0.3];
            expected = -agc.math.adTwist(U).' * (I6 * V);

            actual = agc.paper.coriolis('c2', V, I6, U);

            testCase.verifyEqual(actual, expected, 'AbsTol', 1e-12);
        end

        function errorCovectorUsesPaperAttitudeScale(testCase)
            theta = pi / 3;
            R = [cos(theta), -sin(theta), 0; sin(theta), cos(theta), 0; 0, 0, 1];
            He = [R, [0.3; -0.2; 0.1]; 0, 0, 0, 1];
            KR = diag([2, 3, 5]);
            Kxi = diag([7, 11, 13]);
            expectedER = agc.math.unskew(0.5 * (KR * R - R.' * KR));
            expectedEP = R.' * Kxi * He(1:3, 4);

            [eH, psi] = agc.paper.errors('potential', He, KR, Kxi);

            testCase.verifyEqual(eH, [expectedER; expectedEP], 'AbsTol', 1e-12);
            testCase.verifyGreaterThan(psi, 0);
        end

        function bregmanStepPreservesPositiveDefiniteness(testCase)
            pi = [3; 0.15; -0.1; 0.08; 0.42; 0.51; 0.62; 0.01; -0.03; 0.02];
            J = agc.math.pseudoFromPi(pi);
            G = [2, -1, 0, 3; -1, -4, 2, 0; 0, 2, 1, -2; 3, 0, -2, 5];

            Jnext = agc.paper.adaptation('bregman-step', J, G, 50, 0.2);

            testCase.verifyTrue(agc.math.isSPD(Jnext));
            testCase.verifyEqual(Jnext, Jnext.', 'AbsTol', 1e-12);
        end

        function euclideanStepMatchesGradientUpdate(testCase)
            piHat = [2; 0.1; -0.2; 0.3; 0.4; 0.5; 0.6; 0.01; -0.02; 0.03];
            gradient = [0.2; -0.3; 0.1; 0.4; -0.5; 0.6; -0.7; 0.8; -0.9; 1.0];
            Gamma = diag(1:10) * 1e-2;
            dt = 0.05;

            actual = agc.paper.adaptation('euclidean-step', piHat, gradient, Gamma, dt);
            expected = piHat - dt * Gamma * gradient;

            testCase.verifyEqual(actual, expected, 'AbsTol', 1e-12);
        end

        function regressorMatchesBothPaperFactorizations(testCase)
            pi = [2.1; 0.05; -0.03; 0.02; 0.33; 0.41; 0.49; 0.01; 0.02; -0.01];
            H = eye(4); H(1:3,1:3) = axang2rotm([0.2, -0.3, 0.4, 0.6]);
            V = [0.5; -0.4; 0.1; 0.6; 0.2; -0.7];
            Vr = [-0.2; 0.3; 0.4; 0.1; -0.5; 0.6];
            Vrdot = [0.7; 0.2; -0.1; -0.3; 0.4; 0.5];
            I6 = agc.math.inertiaFromPi(pi);
            g = [0; 0; 9.81];

            for form = ["c1", "c2"]
                Y = agc.paper.regressor(H, V, Vr, Vrdot, g, form);
                Wg = [agc.math.skew(pi(2:4)) * H(1:3,1:3).' * g; pi(1) * H(1:3,1:3).' * g];
                adV = agc.math.adTwist(V);
                adVr = agc.math.adTwist(Vr);
                if form == "c1"
                    coriolisWrench = 0.5 * (I6 * adV * Vr - adVr.' * (I6 * V) - adV.' * (I6 * Vr));
                else
                    coriolisWrench = -adVr.' * (I6 * V);
                end
                expected = I6 * Vrdot + coriolisWrench + Wg;
                testCase.verifyEqual(Y * pi, expected, 'AbsTol', 1e-11);
            end
        end

        function nominalControllerUsesFractionalDissipation(testCase)
            state = struct('H', eye(4), 'V', [0; 0; 0; 0.4; 0; 0]);
            desired = struct('H', eye(4), 'V', zeros(6,1), 'Vdot', zeros(6,1));
            pi = [1.5; 0; 0; 0; 0.2; 0.25; 0.3; 0; 0; 0];
            cfg = struct('mode', 'nominal', 'coriolis', 'c1', 'KR', eye(3), ...
                'Kxi', eye(3), 'Lambda', eye(6), 'kd', 1, 'ks', 2, ...
                'alpha', 0.5, 'gravity', [0; 0; 9.81]);

            [W, diagnostics] = agc.paper.controller(state, desired, cfg, pi, []);

            expectedD = (1 + 2 * norm(diagnostics.s)^(-0.5)) * diagnostics.s;
            expected = agc.math.inertiaFromPi(pi) * diagnostics.VrDot + ...
                agc.paper.coriolis('c1', state.V, agc.math.inertiaFromPi(pi), diagnostics.Vr) + ...
                diagnostics.Wg - expectedD;
            testCase.verifyEqual(W, expected, 'AbsTol', 1e-12);
        end

        function potentialDerivativeMatchesErrorKinematics(testCase)
            He = [axang2rotm([0.3, 0.2, -0.1, 0.4]), [0.2; -0.1; 0.3]; 0, 0, 0, 1];
            Ve = [0.4; -0.5; 0.2; 0.3; 0.1; -0.2];
            KR = diag([2, 3, 4]); Kxi = diag([5, 6, 7]); dt = 1e-7;
            [e0, ~] = agc.paper.errors('potential', He, KR, Kxi);
            Hnext = [He(1:3,1:3) * expm(agc.math.skew(Ve(1:3)) * dt), ...
                He(1:3,4) + He(1:3,1:3) * Ve(4:6) * dt; 0, 0, 0, 1];
            [e1, ~] = agc.paper.errors('potential', Hnext, KR, Kxi);

            derivative = agc.paper.errors('potential-derivative', He, Ve, KR, Kxi);

            testCase.verifyEqual(derivative, (e1 - e0) / dt, 'AbsTol', 1e-5);
        end
    end
end
