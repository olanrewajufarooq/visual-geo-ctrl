classdef TestAdaptationMath < matlab.unittest.TestCase
    %TESTADAPTATIONMATH Mathematical correctness tests for the adaptation package.
    %
    %   Tests the key algebraic identities that the adaptation relies on:
    %     1. paramgenmomentum identity: X(V)*pi == I6*V
    %     2. params2genInertia consistency with getGeneralizedInertia
    %     3. Regressor identity: Y(H,V,VR,VRDot)*pi == W_model(pi)
    %     4. Adaptation direction: one update step moves pi_hat toward pi_true

    properties (Constant)
        % A physically valid parameter vector (hexacopter-like values).
        % pi = [m; hx; hy; hz; Ixx; Iyy; Izz; Ixy; Ixz; Iyz]
        PI_TRUE = [5.0; 0.05; -0.03; 0.02; ...
                   0.20; 0.18; 0.15; 5e-4; -3e-4; 2e-4]

        % A hover-like rotation (slight tilt)
        G_BODY = [0; 0; 9.81]    % gravity in world frame [m/s^2]
    end

    % ------------------------------------------------------------------
    % Helper: build I6 directly from pi without using RBInertia
    % ------------------------------------------------------------------
    methods (Static, Access = private)
        function I6 = buildI6(pi)
            m  = pi(1);
            h  = pi(2:4);
            Iv = pi(5:10);  % [Ixx; Iyy; Izz; Ixy; Ixz; Iyz]
            J  = [Iv(1) Iv(4) Iv(5);
                  Iv(4) Iv(2) Iv(6);
                  Iv(5) Iv(6) Iv(3)];
            hx = fth.se3.hat3(h);
            I6 = [J,   hx;
                  -hx, m*eye(3)];
        end

        function Wg = gravityWrench(H, pi)
            %GRAVITYWRENCH Body-frame gravity wrench for the given pose and pi.
            m   = pi(1);
            h   = pi(2:4);
            R   = H(1:3,1:3);
            g_b = R' * [0; 0; 9.81];
            f_g = m * g_b;
            tau_g = cross(h / m, f_g);   % cross(CoG, f_g) = cross(h/m, m*g_b)
            Wg  = [tau_g; f_g];
        end
    end

    % ------------------------------------------------------------------
    % Test 1: paramgenmomentum identity
    % ------------------------------------------------------------------
    methods (Test)
        function testParamgenmomentumIdentity(testCase)
            %TESTPARAMGENMOMENTUMIDENTITY
            %   X(V) * pi == I6 * V for several velocity vectors.
            pi = TestAdaptationMath.PI_TRUE;
            I6 = fth.ctrl.adapt.AdaptationUtils.params2genInertia(pi);

            velocities = {
                [0.1; -0.2; 0.3; 0.5; -0.3; 0.1], ...
                [0; 0; 0; 0; 0; 0], ...
                [1; 0; 0; 0; 0; 0], ...
                [0; 0; 0; 1; 0; 0], ...
                [0.5; -0.3; 0.1; -0.4; 0.2; 0.8]
            };

            for k = 1:numel(velocities)
                V = velocities{k};
                X = fth.utils.RBDynamics.paramgenmomentum(V);
                err = norm(X * pi - I6 * V);
                testCase.verifyLessThan(err, 1e-12, ...
                    sprintf('paramgenmomentum identity failed for velocity %d', k));
            end
        end

        % ------------------------------------------------------------------
        % Test 2: params2genInertia equals getGeneralizedInertia
        % ------------------------------------------------------------------
        function testParams2genInertiaEqualsGetGeneralizedInertia(testCase)
            %TESTPARAMS2GENINERTIAEQUALSGETGENERALIZEDINERTIA
            %   Both should produce the same I6 for the same physical parameters.
            m      = 4.5;
            CoG    = [0.01; 0.02; -0.01];
            Iparams = [0.20; 0.18; 0.15; 1e-3; -5e-4; 2e-4];
            %           Ixx   Iyy   Izz   Ixy   Ixz    Iyz   (unified order)

            pi = [m; m * CoG(:); Iparams];

            I6_new = fth.ctrl.adapt.AdaptationUtils.params2genInertia(pi);
            I6_old = fth.se3.getGeneralizedInertia(m, Iparams, CoG);

            testCase.verifyEqual(I6_new, I6_old, 'AbsTol', 1e-12, ...
                'params2genInertia and getGeneralizedInertia differ');
        end

        % ------------------------------------------------------------------
        % Test 3: Regressor identity — Y * pi == W_model
        % ------------------------------------------------------------------
        function testRegressorIdentity(testCase)
            %TESTREGRESSORIDENTITY
            %   The regressor must satisfy Y(H,V,VR,VRDot)*pi == W_model(pi)
            %   where W_model = I6*VRDot + C(VR,I6)*VR + Wg.
            %
            %   Critically, the Coriolis uses adjoint(VR)^T (matching the control
            %   law's basicCoriolisFactor), NOT adjoint(V)^T.
            pi    = TestAdaptationMath.PI_TRUE;
            I6    = fth.ctrl.adapt.AdaptationUtils.params2genInertia(pi);

            % Arbitrary but realistic tracking scenario
            H     = eye(4);
            H(1,4) = 0.1; H(2,4) = 0.05; H(3,4) = 0.3;  % slight position offset
            R     = H(1:3,1:3);

            V     = [0.1; -0.15; 0.2; 0.3; -0.1; 0.4];
            VR    = [0.08; -0.12; 0.18; 0.25; -0.09; 0.35];
            VRDot = [0.01; -0.02; 0.03; 0.05; -0.01; 0.02];

            % Ground-truth W_model using VR in the Coriolis (basicCoriolisFactor)
            C_VR  = -fth.se3.adV(VR)' * I6;    % basicCoriolisFactor
            g_b   = R' * [0; 0; 9.81];
            f_g   = pi(1) * g_b;               % m * g_body
            tau_g = cross(pi(2:4) / pi(1), f_g);
            Wg    = [tau_g; f_g];
            W_model = I6 * VRDot + C_VR * VR + Wg;

            % Regressor using adjoint(VR)^T (the CORRECT formulation)
            Y = fth.utils.RBDynamics.paramgenmomentum(VRDot) ...
                - fth.utils.RBDynamics.adjoint(VR)' * fth.utils.RBDynamics.paramgenmomentum(VR);
            Y(4:6, 1)   = Y(4:6, 1)   + g_b;
            Y(1:3, 2:4) = Y(1:3, 2:4) - fth.se3.hat3(g_b);

            err = norm(Y * pi - W_model);
            testCase.verifyLessThan(err, 1e-11, ...
                sprintf('Regressor identity failed: ||Y*pi - W_model|| = %.3e', err));
        end

        function testRegressorWithWrongCoriolisFailsIdentity(testCase)
            %TESTREGRESSORWITHWRONGCORIOLISFAILSIDENTITY
            %   Confirm that using adjoint(V)^T (the bug) breaks the identity.
            %   This test documents that the two formulations differ when V != VR.
            pi    = TestAdaptationMath.PI_TRUE;
            I6    = fth.ctrl.adapt.AdaptationUtils.params2genInertia(pi);

            H     = eye(4);
            V     = [0.1; -0.15; 0.2; 0.3; -0.1; 0.4];
            VR    = [0.08; -0.12; 0.18; 0.25; -0.09; 0.35];  % V != VR
            VRDot = [0.01; -0.02; 0.03; 0.05; -0.01; 0.02];
            g_b   = [0; 0; 9.81];

            C_VR     = -fth.se3.adV(VR)' * I6;
            f_g      = pi(1) * g_b;
            tau_g    = cross(pi(2:4) / pi(1), f_g);
            Wg       = [tau_g; f_g];
            W_model  = I6 * VRDot + C_VR * VR + Wg;

            % Buggy regressor: uses adjoint(V)^T instead of adjoint(VR)^T
            Y_bug = fth.utils.RBDynamics.paramgenmomentum(VRDot) ...
                    - fth.utils.RBDynamics.adjoint(V)' * fth.utils.RBDynamics.paramgenmomentum(VR);
            Y_bug(4:6, 1)   = Y_bug(4:6, 1)   + g_b;
            Y_bug(1:3, 2:4) = Y_bug(1:3, 2:4) - fth.se3.hat3(g_b);

            err_bug = norm(Y_bug * pi - W_model);
            % The error should be nonzero when V != VR
            testCase.verifyGreaterThan(err_bug, 1e-6, ...
                'Expected nonzero error with wrong Coriolis argument when V != VR');
        end

        % ------------------------------------------------------------------
        % Test 4: Adaptation direction (sign test)
        % ------------------------------------------------------------------
        function testAdaptationDirectionEuclidean(testCase)
            %TESTADAPTATIONDIRECTIONEUCLIDEAN
            %   One update step with correct sign (-Gamma) must move pi_hat
            %   strictly closer to pi_true.
            %   This fails if the sign is + (anti-stable law).
            pi_true = TestAdaptationMath.PI_TRUE;
            pi_hat  = pi_true .* [0.85; 0.90; 1.10; 0.95; ...
                                  0.92; 1.08; 0.88; 1.20; 0.80; 1.15];

            % Use a simple hover-like scenario: H = I, V = VR (at reference)
            H     = eye(4);
            V     = [0; 0; 0; 0; 0; 0];
            VR    = [0.05; -0.05; 0; 0.1; 0; 0];   % small reference velocity
            VRDot = [0.02; 0; -0.01; 0; 0.05; 0];
            g_b   = [0; 0; 9.81];

            % Compute regressor (using VR — the correct formulation)
            Y = fth.utils.RBDynamics.paramgenmomentum(VRDot) ...
                - fth.utils.RBDynamics.adjoint(VR)' * fth.utils.RBDynamics.paramgenmomentum(VR);
            Y(4:6, 1)   = Y(4:6, 1)   + g_b;
            Y(1:3, 2:4) = Y(1:3, 2:4) - fth.se3.hat3(g_b);

            % Sliding variable from inertia error: s ≈ Y*(pi_hat - pi_true)/I6_diagonal
            % Use a simple s proportional to the wrench error
            I6_hat = fth.ctrl.adapt.AdaptationUtils.params2genInertia(pi_hat);
            I6_true = fth.ctrl.adapt.AdaptationUtils.params2genInertia(pi_true);
            Wg_hat  = [cross(pi_hat(2:4)/pi_hat(1),  pi_hat(1)*g_b); pi_hat(1)*g_b];
            Wg_true = [cross(pi_true(2:4)/pi_true(1), pi_true(1)*g_b); pi_true(1)*g_b];
            s = (I6_hat - I6_true) * VRDot + ...
                (-fth.se3.adV(VR)' * I6_hat - (-fth.se3.adV(VR)' * I6_true)) * VR + ...
                (Wg_hat - Wg_true);   % wrench error ≈ -Kd*s for an equilibrium

            Gamma = diag([0.36; 0.12; 0.12; 0.12; 0.08; 0.08; 0.12; 0.004; 0.004; 0.004]);
            dt    = 0.005;

            pi_hat_new = pi_hat - Gamma * (Y.' * s) * dt;   % CORRECT: minus sign

            err_before = norm(pi_hat     - pi_true);
            err_after  = norm(pi_hat_new - pi_true);

            testCase.verifyLessThan(err_after, err_before, ...
                sprintf('Adaptation with -Gamma should reduce error: %.6f -> %.6f', ...
                        err_before, err_after));
        end

        function testAdaptationWrongSignIncreasesError(testCase)
            %TESTADAPTATIONWRONGSIGNINCREASEDERROR
            %   One update step with wrong sign (+Gamma) must move pi_hat
            %   AWAY from pi_true (documents the bug).
            pi_true = TestAdaptationMath.PI_TRUE;
            pi_hat  = pi_true .* [0.85; 0.90; 1.10; 0.95; ...
                                  0.92; 1.08; 0.88; 1.20; 0.80; 1.15];

            H     = eye(4);
            V     = [0; 0; 0; 0; 0; 0];
            VR    = [0.05; -0.05; 0; 0.1; 0; 0];
            VRDot = [0.02; 0; -0.01; 0; 0.05; 0];
            g_b   = [0; 0; 9.81];

            Y = fth.utils.RBDynamics.paramgenmomentum(VRDot) ...
                - fth.utils.RBDynamics.adjoint(VR)' * fth.utils.RBDynamics.paramgenmomentum(VR);
            Y(4:6, 1)   = Y(4:6, 1)   + g_b;
            Y(1:3, 2:4) = Y(1:3, 2:4) - fth.se3.hat3(g_b);

            I6_hat  = fth.ctrl.adapt.AdaptationUtils.params2genInertia(pi_hat);
            I6_true = fth.ctrl.adapt.AdaptationUtils.params2genInertia(pi_true);
            Wg_hat  = [cross(pi_hat(2:4)/pi_hat(1),  pi_hat(1)*g_b); pi_hat(1)*g_b];
            Wg_true = [cross(pi_true(2:4)/pi_true(1), pi_true(1)*g_b); pi_true(1)*g_b];
            s = (I6_hat - I6_true) * VRDot + ...
                (-fth.se3.adV(VR)' * I6_hat - (-fth.se3.adV(VR)' * I6_true)) * VR + ...
                (Wg_hat - Wg_true);

            Gamma = diag([0.36; 0.12; 0.12; 0.12; 0.08; 0.08; 0.12; 0.004; 0.004; 0.004]);
            dt    = 0.005;

            pi_hat_bug = pi_hat + Gamma * (Y.' * s) * dt;   % WRONG: plus sign

            err_before = norm(pi_hat     - pi_true);
            err_after  = norm(pi_hat_bug - pi_true);

            testCase.verifyGreaterThan(err_after, err_before, ...
                'Expected +Gamma to increase parameter error (documents the anti-stability bug)');
        end
    end
end
