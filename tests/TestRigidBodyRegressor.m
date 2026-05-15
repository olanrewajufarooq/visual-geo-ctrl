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
    %  Consistency checks: Y*pi == I6*VRDot + C(V,I6)*VR + Wg
    %
    %  Gravity wrench convention (body frame, V=[omega;v]):
    %    Wg(1:3) = R'*vec2tilde(gW)*h   (torque)
    %    Wg(4:6) = m * R'*gW       (force)
    %  where gW is world gravity and h = pi(2:4) is the first moment.
    % ------------------------------------------------------------------ %
    methods (Test)
        function testConsistencyBasic(tc)
            rng(42);
            rbr = fth.ctrl.RigidBodyRegressor(9.81, 'basic');

            pi = tc.randomPi();
            [H, V, VR, VRDot] = tc.randomInputs();

            Y     = rbr.getRegressor(H, V, VR, VRDot);
            Ypred = Y * pi;

            I6      = fth.ctrl.adapt.AdaptationUtils.params2genInertia(pi);
            C_basic = -fth.se3.adV(V).' * I6;
            R       = H(1:3, 1:3);
            gW      = [0; 0; 9.81];
            gB      = R.' * gW;
            m       = pi(1);
            h       = pi(2:4);
            Wg      = [R.' * fth.se3.vec2tilde(gW) * h; m * gB];
            Wref    = I6 * VRDot(:) + C_basic * VR(:) + Wg;

            tc.verifyEqual(Ypred, Wref, 'AbsTol', 1e-10);
        end

        function testConsistencyConsistent(tc)
            rng(43);
            rbr = fth.ctrl.RigidBodyRegressor(9.81, 'consistent');

            pi = tc.randomPi();
            [H, V, VR, VRDot] = tc.randomInputs();

            Y     = rbr.getRegressor(H, V, VR, VRDot);
            Ypred = Y * pi;

            % C(V,I) = 0.5*(I*adV(V) - coadP(I*V) - adV(V)'*I)
            I6     = fth.ctrl.adapt.AdaptationUtils.params2genInertia(pi);
            C_cons = 0.5 * (I6 * fth.se3.adV(V) ...
                          - fth.se3.coadP(I6 * V) ...
                          - fth.se3.adV(V).' * I6);
            R      = H(1:3, 1:3);
            gW     = [0; 0; 9.81];
            gB     = R.' * gW;
            m      = pi(1);
            h      = pi(2:4);
            Wg     = [R.' * fth.se3.vec2tilde(gW) * h; m * gB];
            Wref   = I6 * VRDot(:) + C_cons * VR(:) + Wg;

            tc.verifyEqual(Ypred, Wref, 'AbsTol', 1e-10);
        end
    end

    % ------------------------------------------------------------------ %
    %  Helper methods
    % ------------------------------------------------------------------ %
    methods (Static, Access = private)
        function [H, V, VR, VRDot] = randomInputs()
            % Generate a valid rotation via QR decomposition.
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
            d   = abs(randn(3,1)) + 1;      % Jxx, Jyy, Jzz (diagonal dominant)
            off = randn(3,1) * 0.05;        % Jxy, Jxz, Jyz (small off-diagonal)
            pi  = [m; h; d; off];
        end
    end
end
