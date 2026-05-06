classdef TestWrenchController < matlab.unittest.TestCase
    %TESTWRENCHCONTROLLER Unit tests for WrenchController wrench computation.

    methods (Test)
        function testPDAtHoverEquilibriumCancelsGravity(testCase)
            %At hover (identity pose, zero velocity), PD wrench should
            %cancel gravity and have zero error terms.
            cfg = testCase.buildCfg('PD');
            ctrl = vt.ctrl.WrenchController(cfg);
            Hd = eye(4); Hd(3,4) = 5;
            H = Hd;
            V = zeros(6,1);
            Vd = zeros(6,1);
            W = ctrl.computeWrench(Hd, H, Vd, V);
            % At equilibrium with zero error, wrench should only cancel gravity.
            % Gravity wrench in body frame for identity rotation:
            %   f_g = m * [0;0;-g], tau_g = cross(CoG, f_g)
            m = cfg.vehicle.m;
            g = cfg.vehicle.g;
            CoG = cfg.vehicle.CoG;
            f_g = m * [0; 0; -g];
            tau_g = cross(CoG, f_g);
            Wg = [tau_g; f_g];
            % W = -Wg (gravity cancellation) with zero error contributions
            testCase.verifyEqual(W, -Wg, 'AbsTol', 1e-10);
        end

        function testPDWrenchIncreasesWithPoseError(testCase)
            %Larger pose error should produce larger wrench magnitude.
            cfg = testCase.buildCfg('PD');
            ctrl = vt.ctrl.WrenchController(cfg);
            Hd = eye(4); Hd(3,4) = 5;
            V = zeros(6,1);
            Vd = zeros(6,1);

            % Small position error
            H1 = Hd; H1(1,4) = H1(1,4) + 0.1;
            W1 = ctrl.computeWrench(Hd, H1, Vd, V);

            % Larger position error
            H2 = Hd; H2(1,4) = H2(1,4) + 1.0;
            W2 = ctrl.computeWrench(Hd, H2, Vd, V);

            testCase.verifyGreaterThan(norm(W2), norm(W1));
        end

        function testFeedLinIncludesCoriolis(testCase)
            %FeedLin wrench should differ from PD when velocity is nonzero
            %(Coriolis term is active).
            cfgPD = testCase.buildCfg('PD');
            cfgFL = testCase.buildCfg('FeedLin');
            ctrlPD = vt.ctrl.WrenchController(cfgPD);
            ctrlFL = vt.ctrl.WrenchController(cfgFL);

            Hd = eye(4); Hd(3,4) = 5;
            H = Hd;
            V = [0.1; 0.2; 0.3; 0.4; 0.5; 0.6];
            Vd = zeros(6,1);

            W_pd = ctrlPD.computeWrench(Hd, H, Vd, V);
            W_fl = ctrlFL.computeWrench(Hd, H, Vd, V);
            % The difference should be the Coriolis cancellation term -C
            I6 = cfg_I6(cfgFL);
            C = vt.se3.adV(V)' * I6 * V;
            testCase.verifyEqual(W_fl - W_pd, -C, 'AbsTol', 1e-10);
        end

        function testFeedforwardIncludesAcceleration(testCase)
            %Feedforward wrench should include reference acceleration term.
            cfgFL = testCase.buildCfg('FeedLin');
            cfgFF = testCase.buildCfg('Feedforward');
            ctrlFL = vt.ctrl.WrenchController(cfgFL);
            ctrlFF = vt.ctrl.WrenchController(cfgFF);

            Hd = eye(4); Hd(3,4) = 5;
            H = Hd;
            V = zeros(6,1);
            Vd = zeros(6,1);
            Ades = [0; 0; 0; 0; 0; 1];

            W_fl = ctrlFL.computeWrench(Hd, H, Vd, V, Ades);
            W_ff = ctrlFF.computeWrench(Hd, H, Vd, V, Ades);
            % At zero error (H==Hd, V==Vd==0), feedforward adds I6 * Ades
            I6 = cfg_I6(cfgFF);
            ff_term = I6 * Ades;
            testCase.verifyEqual(W_ff - W_fl, ff_term, 'AbsTol', 1e-10);
        end

        function testWrenchIsFinite(testCase)
            %Wrench output should always be finite for valid inputs.
            cfg = testCase.buildCfg('Feedforward');
            ctrl = vt.ctrl.WrenchController(cfg);
            Hd = eye(4); Hd(3,4) = 5;
            H = Hd; H(1,4) = 0.5; H(2,4) = -0.3;
            V = [0.01; -0.02; 0.03; 0.1; -0.1; 0.2];
            Vd = [0; 0; 0; 0; 0; 0.1];
            Ades = [0; 0; 0; 0; 0; 0.05];
            W = ctrl.computeWrench(Hd, H, Vd, V, Ades);
            testCase.verifyTrue(all(isfinite(W)));
        end

        function testWrenchDimensionIs6x1(testCase)
            %Wrench must be 6x1 regardless of controller mode.
            modes = {'PD', 'FeedLin', 'Feedforward'};
            for i = 1:numel(modes)
                cfg = testCase.buildCfg(modes{i});
                ctrl = vt.ctrl.WrenchController(cfg);
                W = ctrl.computeWrench(eye(4), eye(4), zeros(6,1), zeros(6,1));
                testCase.verifySize(W, [6, 1]);
            end
        end

        function testGetEstimateReturnsEmptyForNoAdaptation(testCase)
            cfg = testCase.buildCfg('PD');
            ctrl = vt.ctrl.WrenchController(cfg);
            [m_hat, cog_hat, I_hat] = ctrl.getEstimate();
            testCase.verifyEmpty(m_hat);
            testCase.verifyEmpty(cog_hat);
            testCase.verifyEmpty(I_hat);
        end
    end

    methods (Static, Access = private)
        function cfg = buildCfg(mode)
            %BUILDCFG Create a minimal Config for controller testing.
            cfg = vt.config.Config();
            cfg.setTrajectory('hover');
            cfg.setController(mode, 'liealgebra');
            cfg.done();
        end
    end
end

function I6 = cfg_I6(cfg)
    %CFG_I6 Extract generalized inertia from config.
    I6 = cfg.vehicle.I6;
end
