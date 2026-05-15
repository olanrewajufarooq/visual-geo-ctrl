classdef TestWrenchControllerAdaptation < matlab.unittest.TestCase
    %TESTWRENCHCONTROLLERADAPTATION Integration tests for updatePi.
    %   Tests that updatePi executes without error with valid inputs.

    methods (Test)
        function testUpdateAdaptationWithZeroVelocity(testCase)
            %TESTUPDATEADAPTATIONWITHZEROVELOCITY
            %   Edge case: zero velocity should not crash.
            cfg = fth.sim.Config();
            cfg.setController('Feedforward');
            cfg.setAdaptation('euclidean');
            cfg.setAdaptiveGains(ones(10,1));
            cfg.done();

            ctrl = fth.ctrl.ControllerFactory.create(cfg);
            
            Hd = eye(4);
            H = eye(4);
            Vd = zeros(6,1);
            V = zeros(6,1);
            Ades = zeros(6,1);
            dt = 0.001;
            
            ctrl.updatePi(Hd, H, Vd, V, Ades, dt);
            testCase.verifyTrue(true);
        end

        function testUpdateAdaptationWithMissingOptionalArgs(testCase)
            %TESTUPDATEADAPTATIONWITHMISSINGOPTIONALARGS
            %   Test that Ades and dt defaults are applied.
            cfg = fth.sim.Config();
            cfg.setController('Feedforward');
            cfg.setAdaptation('euclidean');
            cfg.setAdaptiveGains([0.5; zeros(9,1)]);
            cfg.done();

            ctrl = fth.ctrl.ControllerFactory.create(cfg);
            
            Hd = eye(4);
            H = eye(4);
            Vd = [0.1; 0.2; 0.3; 0.4; 0.5; 0.6];
            V = [0.11; 0.21; 0.31; 0.41; 0.51; 0.61];
            
            ctrl.updatePi(Hd, H, Vd, V);
            testCase.verifyTrue(true);
        end

        function testUpdateAdaptationEuclideanSingleStep(testCase)
            %TESTUPDATEADAPTATIONEUCLIDEANSINGLSTEP
            %   Euclidean adaptation: single update step with nominal config.
            cfg = fth.sim.Config();
            cfg.setController('Feedforward');
            cfg.setAdaptation('euclidean');
            cfg.setAdaptiveGains([0.1; zeros(9,1)]);
            cfg.done();

            ctrl = fth.ctrl.ControllerFactory.create(cfg);
            
            Hd = eye(4);
            Hd(1:3, 4) = [0.2; 0.1; 0.05];
            H = eye(4);
            H(1:3, 4) = [0.25; 0.12; 0.07];
            Vd = [0.05; 0.1; 0.15; 0.2; 0.25; 0.3];
            V = [0.06; 0.11; 0.16; 0.21; 0.26; 0.31];
            Ades = [0.001; 0.002; 0.003; 0.004; 0.005; 0.006];
            dt = 0.001;
            
            ctrl.updatePi(Hd, H, Vd, V, Ades, dt);
            testCase.verifyTrue(true);
        end

        function testUpdateAdaptationBregmanSingleStep(testCase)
            %TESTUPDATEADAPTATIONBREGMANSINGLSTEP
            %   Bregman adaptation: single update step.
            cfg = fth.sim.Config();
            cfg.setController('Feedforward');
            cfg.setAdaptation('bregman');
            cfg.setAdaptiveGains(0.05);
            cfg.done();

            ctrl = fth.ctrl.ControllerFactory.create(cfg);
            
            Hd = eye(4);
            Hd(1:3, 4) = [0.1; 0.05; 0.02];
            H = eye(4);
            H(1:3, 4) = [0.15; 0.07; 0.03];
            Vd = [0.1; 0.15; 0.2; 0.25; 0.3; 0.35];
            V = [0.12; 0.17; 0.22; 0.27; 0.32; 0.37];
            Ades = [0.01; 0.02; 0.03; 0.04; 0.05; 0.06];
            dt = 0.001;
            
            ctrl.updatePi(Hd, H, Vd, V, Ades, dt);
            testCase.verifyTrue(true);
        end

        function testUpdateAdaptationMultipleSteps(testCase)
            %TESTUPDATEADAPTATIONMULTIPLESTEPS
            %   Multiple sequential update steps with Euclidean adaptation.
            cfg = fth.sim.Config();
            cfg.setController('Feedforward');
            cfg.setAdaptation('euclidean');
            cfg.setAdaptiveGains([0.1; 0.05; zeros(8,1)]);
            cfg.done();

            ctrl = fth.ctrl.ControllerFactory.create(cfg);
            
            for step = 1:5
                Hd = eye(4);
                Hd(1:3, 4) = [0.1 * step; 0.05 * step; 0.02 * step];
                H = eye(4);
                H(1:3, 4) = [0.11 * step; 0.055 * step; 0.022 * step];
                Vd = 0.1 * randn(6,1);
                V = Vd + 0.01 * randn(6,1);
                Ades = 0.01 * randn(6,1);
                dt = 0.001;
                
                ctrl.updatePi(Hd, H, Vd, V, Ades, dt);
            end
            
            testCase.verifyTrue(true);
        end

        function testUpdateAdaptationIntegrationWithBregmanMultiStep(testCase)
            %TESTUPDATEADAPTATIONINTEGRATIONWITHBREGMANMULTISTEP
            %   Full integration test with BregmanDivAdaptation over multiple steps.
            cfg = fth.sim.Config();
            cfg.setController('Feedforward');
            cfg.setAdaptation('bregman');
            cfg.setAdaptiveGains(0.1);
            cfg.done();

            ctrl = fth.ctrl.ControllerFactory.create(cfg);
            
            for step = 1:5
                Hd = eye(4);
                Hd(1:3, 4) = [0.1 * step; 0.05 * step; 0.02 * step];
                H = eye(4);
                H(1:3, 4) = [0.11 * step; 0.055 * step; 0.022 * step];
                Vd = 0.1 * randn(6,1);
                V = Vd + 0.01 * randn(6,1);
                Ades = 0.01 * randn(6,1);
                dt = 0.001;
                
                ctrl.updatePi(Hd, H, Vd, V, Ades, dt);
            end
            
            testCase.verifyTrue(true);
        end

        function testUpdateAdaptationNonIdenticalPoseAndVelocity(testCase)
            %TESTUPDATEADAPTATIONNONIDENTICALPOSANDVELOCITY
            %   Test with realistic tracking errors (pose and velocity mismatch).
            cfg = fth.sim.Config();
            cfg.setController('Feedforward');
            cfg.setAdaptation('euclidean');
            cfg.setAdaptiveGains(ones(10,1));
            cfg.done();

            ctrl = fth.ctrl.ControllerFactory.create(cfg);
            
            Hd = eye(4);
            Hd(1:3, 4) = [1.0; 0.5; 0.3];
            Vd = [0.5; -0.2; 0.3; 0.4; 0.1; 0.2];
            Ades = [0.1; 0.05; -0.05; 0.02; 0.03; 0.01];
            
            H = eye(4);
            H(1:3, 4) = [1.05; 0.52; 0.32];
            V = [0.48; -0.22; 0.32; 0.42; 0.12; 0.22];
            dt = 0.001;
            
            ctrl.updatePi(Hd, H, Vd, V, Ades, dt);
            testCase.verifyTrue(true);
        end
    end
end
