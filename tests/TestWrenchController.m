classdef TestWrenchController < matlab.unittest.TestCase
    %TESTWRENCHCONTROLLER Unit tests for WrenchController wrench computation.
    %   Tests the composite-variable control law with the new potential interface.

    methods (Test)
        function testAtHoverEquilibriumCancelsGravity(testCase)
            %At hover (desired == actual, zero velocity, zero acceleration),
            %the wrench should equal gravity cancellation only.
            %   s=0, VR=0, VRDot=0 => W = -Wg
            cfg  = testCase.buildCfg('log');
            ctrl = fth.ctrl.WrenchController(cfg);
            Hd = eye(4); Hd(3,4) = 5;
            H  = Hd;
            V  = zeros(6,1);
            Vd = zeros(6,1);

            W = ctrl.computeWrench(Hd, H, Vd, V);

            m    = cfg.vehicle.m;
            g    = cfg.vehicle.g;
            CoG  = cfg.vehicle.CoG;
            gW = [0; 0; g];
            h  = m * CoG(:);
            Wg = [fth.se3.hat3(gW) * h; m * gW];
            testCase.verifyEqual(W, Wg, 'AbsTol', 1e-10);
        end

        function testWrenchIncreasesWithPoseError(testCase)
            %Larger pose error should produce a larger wrench magnitude.
            cfg  = testCase.buildCfg('log');
            ctrl = fth.ctrl.WrenchController(cfg);
            Hd = eye(4); Hd(3,4) = 5;
            V  = zeros(6,1);
            Vd = zeros(6,1);

            H1 = Hd; H1(1,4) = H1(1,4) + 0.1;
            W1 = ctrl.computeWrench(Hd, H1, Vd, V);

            H2 = Hd; H2(1,4) = H2(1,4) + 1.0;
            W2 = ctrl.computeWrench(Hd, H2, Vd, V);

            testCase.verifyGreaterThan(norm(W2), norm(W1));
        end

        function testWrenchIsFinite(testCase)
            %Wrench output must be finite for valid inputs.
            cfg  = testCase.buildCfg('log');
            ctrl = fth.ctrl.WrenchController(cfg);
            Hd = eye(4); Hd(3,4) = 5;
            H  = Hd; H(1,4) = 0.5; H(2,4) = -0.3;
            V  = [0.01; -0.02; 0.03; 0.1; -0.1; 0.2];
            Vd = [0; 0; 0; 0; 0; 0.1];
            Ades = [0; 0; 0; 0; 0; 0.05];
            W = ctrl.computeWrench(Hd, H, Vd, V, Ades);
            testCase.verifyTrue(all(isfinite(W)));
        end

        function testWrenchDimensionIs6x1(testCase)
            %Wrench must always be 6x1.
            for potType = {'log', 'inertia-gain', 'sym-inv'}
                cfg  = testCase.buildCfg(potType{1});
                ctrl = fth.ctrl.WrenchController(cfg);
                W = ctrl.computeWrench(eye(4), eye(4), zeros(6,1), zeros(6,1));
                testCase.verifySize(W, [6, 1], ...
                    sprintf('Wrench must be 6x1 for potType ''%s''.', potType{1}));
            end
        end

        function testLogPotentialErrorIs6x1(testCase)
            %getPotentialError and derivative must return 6x1 for logPotential.
            Kp = diag([5.5 5.5 5.5 5.5 5.5 5.5]);
            pot = fth.ctrl.potential.logPotential(Kp);
            Hd = eye(4); Hd(3,4) = 5;
            H  = Hd; H(1,4) = 0.3;
            V  = [0.1; 0; 0; 0; 0; 0.2];
            Vd = zeros(6,1);

            eH    = pot.getPotentialError(Hd, H);
            eHDot = pot.getPotentialErrorDerivative(Hd, H, Vd, V);

            testCase.verifySize(eH,    [6, 1]);
            testCase.verifySize(eHDot, [6, 1]);
            testCase.verifyTrue(all(isfinite(eH)));
            testCase.verifyTrue(all(isfinite(eHDot)));
        end

        function testInertiaGainPotentialErrorIs6x1(testCase)
            %inertiaGainPotential must return finite 6x1 eH and eHDot.
            K_R  = diag([5.5 5.5 5.5]);
            K_xi = diag([5.5 5.5 5.5]);
            pot  = fth.ctrl.potential.inertiaGainPotential(K_R, K_xi);
            Hd   = eye(4); Hd(3,4) = 5;
            H    = Hd; H(1,4) = 0.3;
            V    = [0.1; 0; 0; 0; 0; 0.2];
            Vd   = zeros(6,1);

            eH    = pot.getPotentialError(Hd, H);
            eHDot = pot.getPotentialErrorDerivative(Hd, H, Vd, V);

            testCase.verifySize(eH,    [6, 1]);
            testCase.verifySize(eHDot, [6, 1]);
            testCase.verifyTrue(all(isfinite(eH)));
            testCase.verifyTrue(all(isfinite(eHDot)));
        end

        function testSymInvPotentialErrorIs6x1(testCase)
            %symInvPotential must return finite 6x1 eH and eHDot.
            K_R  = diag([5.5 5.5 5.5]);
            K_xi = diag([5.5 5.5 5.5]);
            pot  = fth.ctrl.potential.symInvPotential(K_R, K_xi);
            Hd   = eye(4); Hd(3,4) = 5;
            H    = Hd; H(1,4) = 0.3;
            V    = [0.1; 0; 0; 0; 0; 0.2];
            Vd   = zeros(6,1);

            eH    = pot.getPotentialError(Hd, H);
            eHDot = pot.getPotentialErrorDerivative(Hd, H, Vd, V);

            testCase.verifySize(eH,    [6, 1]);
            testCase.verifySize(eHDot, [6, 1]);
            testCase.verifyTrue(all(isfinite(eH)));
            testCase.verifyTrue(all(isfinite(eHDot)));
        end

        function testBodyGainDerivativeThrows(testCase)
            %bodyGainPotential.getPotentialErrorDerivative must error.
            K_R  = diag([5.5 5.5 5.5]);
            K_xi = diag([5.5 5.5 5.5]);
            pot  = fth.ctrl.potential.bodyGainPotential(K_R, K_xi);
            testCase.verifyError( ...
                @() pot.getPotentialErrorDerivative(eye(4), eye(4), zeros(6,1), zeros(6,1)), ...
                'fth:bodyGainPotential:NotImplemented');
        end

        function testRefGainDerivativeThrows(testCase)
            %refGainPotential.getPotentialErrorDerivative must error.
            K_R  = diag([5.5 5.5 5.5]);
            K_xi = diag([5.5 5.5 5.5]);
            pot  = fth.ctrl.potential.refGainPotential(K_R, K_xi);
            testCase.verifyError( ...
                @() pot.getPotentialErrorDerivative(eye(4), eye(4), zeros(6,1), zeros(6,1)), ...
                'fth:refGainPotential:NotImplemented');
        end

        function testGetEstimateReturnsEmptyForNoAdaptation(testCase)
            cfg  = testCase.buildCfg('log');
            ctrl = fth.ctrl.WrenchController(cfg);
            [m_hat, cog_hat, I_hat] = ctrl.getEstimate();
            testCase.verifyEmpty(m_hat);
            testCase.verifyEmpty(cog_hat);
            testCase.verifyEmpty(I_hat);
        end

        function testComputeWrenchUsesRegressor(testCase)
            %W is finite and 6x1 for non-trivial state (smoke + size check).
            cfg  = testCase.buildCfg('log');
            ctrl = fth.ctrl.WrenchController(cfg);

            [Q, ~] = qr(randn(3));
            if det(Q) < 0; Q(:,1) = -Q(:,1); end
            H  = eye(4); H(1:3,1:3) = Q; H(1:3,4) = [1; -0.5; 3];
            Hd = eye(4); Hd(1:3,4)  = [0; 0; 5];
            V  = [0.1; -0.05; 0.2; 0.3; -0.1; 0.5];
            Vd = zeros(6,1);

            W = ctrl.computeWrench(Hd, H, Vd, V);
            testCase.verifySize(W, [6, 1]);
            testCase.verifyTrue(all(isfinite(W)));
        end

        function testCoriolisFactorIs6x6(testCase)
            %basicCoriolisFactor.getCoriolisFactor must return 6x6.
            cf = fth.ctrl.coriolis.basicCoriolisFactor();
            I6 = eye(6);
            VR = [0.1; 0.2; 0.3; 0.4; 0.5; 0.6];
            C  = cf.getCoriolisFactor(VR, I6);
            testCase.verifySize(C, [6, 6]);
            testCase.verifyTrue(all(isfinite(C(:))));
        end
    end

    methods (Static, Access = private)
        function cfg = buildCfg(potType)
            %BUILDCFG Create a minimal Config for controller testing.
            cfg = fth.sim.Config();
            cfg.setTrajectory('hover');
            cfg.setController('composite', potType);
            cfg.done();
        end
    end
end
