classdef TestRigidBodyRegressor < matlab.unittest.TestCase
    %TESTRIGIDBODYREGRESSOR Unit tests for fth.ctrl.RigidBodyRegressor.

    % ------------------------------------------------------------------ %
    %  Constructor tests
    % ------------------------------------------------------------------ %
    methods (Test)
        function testDefaultConstructor(tc)
            rbr = fth.ctrl.RigidBodyRegressor();
            tc.verifyEqual(rbr.CoriolisForm, 'basic');
            tc.verifyEqual(rbr.GravityWorld, [0; 0; 9.81]);
        end

        function testScalarGravity(tc)
            rbr = fth.ctrl.RigidBodyRegressor(5.0);
            tc.verifyEqual(rbr.GravityWorld, [0; 0; 5.0]);
        end

        function testVectorGravity(tc)
            g = [0; 9.81; 0];
            rbr = fth.ctrl.RigidBodyRegressor(g);
            tc.verifyEqual(rbr.GravityWorld, g);
        end

        function testInvalidCoriolisFormThrows(tc)
            tc.verifyError( ...
                @() fth.ctrl.RigidBodyRegressor(9.81, 'bogus'), ...
                'RigidBodyRegressor:InvalidCoriolisForm');
        end

        function testInvalidGravityThrows(tc)
            tc.verifyError( ...
                @() fth.ctrl.RigidBodyRegressor(ones(5,1)), ...
                'RigidBodyRegressor:InvalidGravity');
        end
    end

    % ------------------------------------------------------------------ %
    %  getRegressor size / smoke tests
    % ------------------------------------------------------------------ %
    methods (Test)
        function testGetRegressorSizeBasic(tc)
            rbr = fth.ctrl.RigidBodyRegressor(9.81, 'basic');
            [H, V, VR, VRDot] = tc.randomInputs();
            Y = rbr.getRegressor(H, V, VR, VRDot);
            tc.verifySize(Y, [6, 10]);
        end

        function testGetRegressorSizeConsistent(tc)
            rbr = fth.ctrl.RigidBodyRegressor(9.81, 'consistent');
            [H, V, VR, VRDot] = tc.randomInputs();
            Y = rbr.getRegressor(H, V, VR, VRDot);
            tc.verifySize(Y, [6, 10]);
        end
    end

    % ------------------------------------------------------------------ %
    %  Consistency check: Y*pi == I6*VRDot + C_basic(V,I6)*VR + Wg
    % ------------------------------------------------------------------ %
    methods (Test)
        function testConsistencyBasic(tc)
            rng(42);
            rbr = fth.ctrl.RigidBodyRegressor(9.81, 'basic');

            % Random valid pi (ensure positive-definite inertia)
            pi = tc.randomPi();

            [H, V, VR, VRDot] = tc.randomInputs();

            % Regressor prediction
            Y = rbr.getRegressor(H, V, VR, VRDot);
            Ypred = Y * pi;

            % Ground-truth wrench
            I6     = fth.utils.RBInertia.params2genInertia(pi);
            C_basic = -fth.se3.adV(V).' * I6;
            R      = H(1:3, 1:3);
            gB     = R.' * [0; 0; 9.81];
            m      = pi(1);
            h      = pi(2:4);           % first moment (= m * CoG)
            Wg     = [cross(h, gB); m * gB];
            Wref   = I6 * VRDot(:) + C_basic * VR(:) + Wg;

            tc.verifyEqual(Ypred, Wref, 'AbsTol', 1e-10);
        end
    end

        function testConsistencyConsistent(tc)
            rng(43);
            rbr = fth.ctrl.RigidBodyRegressor(9.81, 'consistent');

            pi = tc.randomPi();
            [H, V, VR, VRDot] = tc.randomInputs();

            Y = rbr.getRegressor(H, V, VR, VRDot);
            Ypred = Y * pi;

            % Ground-truth: consistent Coriolis C = 0.5*(I*adV(V) - adV(I*V) - adV(V)'*I)
            I6     = fth.utils.RBInertia.params2genInertia(pi);
            C_cons = 0.5 * (I6 * fth.se3.adV(V) - fth.se3.adV(I6*V) - fth.se3.adV(V).' * I6);
            R      = H(1:3, 1:3);
            gB     = R.' * [0; 0; 9.81];
            m      = pi(1);
            h      = pi(2:4);
            Wg     = [cross(h, gB); m * gB];
            Wref   = I6 * VRDot(:) + C_cons * VR(:) + Wg;

            tc.verifyEqual(Ypred, Wref, 'AbsTol', 1e-10);
        end
    end

    % ------------------------------------------------------------------ %
    %  Helper methods
    % ------------------------------------------------------------------ %
    methods (Static, Access = private)
        function [H, V, VR, VRDot] = randomInputs()
            % Generate a valid rotation via QR decomposition
            [Q, ~] = qr(randn(3));
            if det(Q) < 0; Q(:,1) = -Q(:,1); end
            H = eye(4);
            H(1:3,1:3) = Q;
            H(1:3,4)   = randn(3,1);

            V     = randn(6,1);
            VR    = randn(6,1);
            VRDot = randn(6,1);
        end

        function pi = randomPi()
            % Build a physically consistent pi with PD inertia tensor.
            m   = abs(randn) + 1;           % positive mass
            h   = randn(3,1) * 0.1;         % small first moment
            % Diagonal-dominant inertia
            d   = abs(randn(3,1)) + 1;      % Jxx, Jyy, Jzz
            off = randn(3,1) * 0.05;        % Jxy, Jxz, Jyz (small)
            pi  = [m; h; d; off];
        end
    end
end
