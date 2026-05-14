classdef TestEstimateInitialization < matlab.unittest.TestCase
    %TESTESTIMATEINITIALIZATION Unit tests for estimate init mode behavior.

    methods (Test)
        function testWrenchControllerSetEstimatePiUpdatesEuclideanAdaptation(testCase)
            cfg = fth.sim.Config();
            cfg.setController('Feedforward');
            cfg.setAdaptation('euclidean');
            cfg.setAdaptiveGains(ones(10,1));
            cfg.done();

            ctrl = fth.ctrl.WrenchController(cfg);
            % New pi format: [m; hx; hy; hz; Ixx; Iyy; Izz; Ixy; Ixz; Iyz]
            m = 7;
            h = [8; 9; 10];
            % Unified Iparams order: [Ixx; Iyy; Izz; Ixy; Ixz; Iyz]
            Iparams = [1; 2; 3; 4; 5; 6];
            pi_input = [m; h; Iparams];
            
            ctrl.setEstimatePi(pi_input);
            [m_hat, cog_hat, I_hat] = ctrl.getEstimate();

            testCase.verifyEqual(m_hat, m);
            testCase.verifyEqual(h(:), h, 'AbsTol', 1e-12);
            testCase.verifyEqual(cog_hat(:), h ./ m, 'AbsTol', 1e-12);
            testCase.verifyEqual(I_hat(:), Iparams, 'AbsTol', 1e-12);
        end

        function testEuclideanAdaptationRegressorUsesPositiveCoriolisSign(testCase)
            cfg = fth.sim.Config();
            cfg.setController('Feedforward');
            cfg.setAdaptation('euclidean');
            cfg.setAdaptiveGains([1; zeros(9,1)]);
            cfg.setAdaptationParams(0.001);
            cfg.done();

            adapt = fth.ctrl.adapt.EuclideanAdaptation(cfg);
            Hd = eye(4);
            H = eye(4);
            Vd = zeros(6,1);
            V = [0.3; -0.2; 0.1; 0.4; -0.5; 0.6];
            Ades = zeros(6,1);
            dt = 0.001;
            s = zeros(6,1);     % Composite sliding variable
            VR = zeros(6,1);    % Reference velocity
            VRDot = zeros(6,1); % Reference acceleration

            [~, ~, I_before] = adapt.getEstimate();
            adapt.update(Hd, H, Vd, V, Ades, dt, s, VR, VRDot);
            [~, ~, I_after] = adapt.getEstimate();

            B1 = zeros(6,6);
            B1(1,1) = 1;
            expectedColumn = fth.se3.adV(V)' * B1 * V;
            expectedDelta = dt * (expectedColumn.' * V);

            testCase.verifyEqual(I_after(1) - I_before(1), expectedDelta, 'AbsTol', 1e-12);
        end

        function testEuclideanAdaptationBasisMatchesGeneralizedInertiaUtility(testCase)
            cfg = fth.sim.Config();
            cfg.setController('Feedforward');
            cfg.setAdaptation('euclidean');
            cfg.done();

            adapt = fth.ctrl.adapt.EuclideanAdaptation(cfg);
            params = adapt.getParams();
            theta = [cfg.vehicle.I_params(:); cfg.vehicle.m; cfg.vehicle.m * cfg.vehicle.CoG(:)];

            B = cell(10,1);
            for k = 1:3
                E = zeros(3,3); E(k,k) = 1;
                G = zeros(6,6); G(1:3,1:3) = E;
                B{k} = G;
            end
            pairs = [1 2; 2 3; 1 3];
            for idx = 1:3
                i = pairs(idx,1); j = pairs(idx,2);
                E = zeros(3,3); E(i,j) = 1; E(j,i) = 1;
                G = zeros(6,6); G(1:3,1:3) = E;
                B{3+idx} = G;
            end
            G = zeros(6,6); G(4:6,4:6) = eye(3);
            B{7} = G;
            for ax = 1:3
                e = zeros(3,1); e(ax) = 1;
                S = fth.se3.hat3(e);
                G = zeros(6,6);
                G(1:3,4:6) = S;
                G(4:6,1:3) = -S;
                B{7+ax} = G;
            end

            I6_basis = zeros(6,6);
            for i = 1:10
                I6_basis = I6_basis + theta(i) * B{i};
            end

            testCase.verifyEqual(params.I6, I6_basis, 'AbsTol', 1e-12);
        end

        function testSetEstimateInitializationAcceptsFixedHigher(testCase)
            cfg = fth.sim.Config();
            cfg.setParamInit('vehicle-plus-payload-higher');
            testCase.verifyEqual(cfg.controller.paramInit.mode, 'vehicle-plus-payload-higher');
        end

        function testSetEstimateInitializationCustomVectorStoredAsFixed(testCase)
            cfg = fth.sim.Config();
            theta = 1:10;
            cfg.setParamInit(theta);
            testCase.verifyEqual(cfg.controller.paramInit.mode, 'custom');
            testCase.verifyEqual(cfg.controller.paramInit.spec, theta(:));
        end

        function testSetEstimateInitializationRandomWithSeedSpecStored(testCase)
            cfg = fth.sim.Config();
            cfg.setParamInit('random', 1234);
            testCase.verifyEqual(cfg.controller.paramInit.mode, 'random');
            testCase.verifyEqual(cfg.controller.paramInit.spec, 1234);
        end

        function testSetEstimateInitializationFixedHigherWithExplicitSpec(testCase)
            cfg = fth.sim.Config();
            theta = (21:30).';
            cfg.setParamInit('vehicle-plus-payload-higher', theta);
            testCase.verifyEqual(cfg.controller.paramInit.mode, 'vehicle-plus-payload-higher');
            testCase.verifyEqual(cfg.controller.paramInit.spec, theta);
        end
    end
end
