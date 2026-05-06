classdef TestEstimateInitialization < matlab.unittest.TestCase
    %TESTESTIMATEINITIALIZATION Unit tests for estimate init mode behavior.

    methods (Test)
        function testWrenchControllerSetEstimateThetaUpdatesEuclideanAdaptation(testCase)
            cfg = vt.config.Config();
            cfg.setController('Feedforward');
            cfg.setAdaptation('euclidean');
            cfg.setAdaptiveGains(ones(10,1));
            cfg.done();

            ctrl = vt.ctrl.WrenchController(cfg);
            theta = (1:10).';
            ctrl.setEstimateTheta(theta);
            [m_hat, cog_hat, I_hat] = ctrl.getEstimate();

            testCase.verifyEqual(I_hat(:), theta(1:6));
            testCase.verifyEqual(m_hat, theta(7));
            testCase.verifyEqual(cog_hat(:), theta(8:10) ./ theta(7), 'AbsTol', 1e-12);
        end

        function testEuclideanAdaptationRegressorUsesPositiveCoriolisSign(testCase)
            cfg = vt.config.Config();
            cfg.setController('Feedforward');
            cfg.setAdaptation('euclidean');
            cfg.setAdaptiveGains([1; zeros(9,1)]);
            cfg.setAdaptationParams(0.001);
            cfg.done();

            adapt = vt.ctrl.adapt.EuclideanAdaptation(cfg);
            Hd = eye(4);
            H = eye(4);
            Vd = zeros(6,1);
            V = [0.3; -0.2; 0.1; 0.4; -0.5; 0.6];
            Ades = zeros(6,1);
            dt = 0.001;

            [~, ~, I_before] = adapt.getEstimate();
            adapt.update(Hd, H, Vd, V, Ades, dt);
            [~, ~, I_after] = adapt.getEstimate();

            B1 = zeros(6,6);
            B1(1,1) = 1;
            expectedColumn = vt.se3.adV(V)' * B1 * V;
            expectedDelta = dt * (expectedColumn.' * V);

            testCase.verifyEqual(I_after(1) - I_before(1), expectedDelta, 'AbsTol', 1e-12);
        end

        function testEuclideanAdaptationBasisMatchesGeneralizedInertiaUtility(testCase)
            cfg = vt.config.Config();
            cfg.setController('Feedforward');
            cfg.setAdaptation('euclidean');
            cfg.done();

            adapt = vt.ctrl.adapt.EuclideanAdaptation(cfg);
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
                S = vt.se3.hat3(e);
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
            cfg = vt.config.Config();
            cfg.setEstimateInitialization('fixed-higher');
            testCase.verifyEqual(cfg.controller.estimateInitialization.mode, 'fixed-higher');
        end

        function testSetEstimateInitializationCustomVectorStoredAsFixed(testCase)
            cfg = vt.config.Config();
            theta = 1:10;
            cfg.setEstimateInitialization(theta);
            testCase.verifyEqual(cfg.controller.estimateInitialization.mode, 'fixed');
            testCase.verifyEqual(cfg.controller.estimateInitialization.spec, theta(:));
        end

        function testSetEstimateInitializationRandomWithSeedSpecStored(testCase)
            cfg = vt.config.Config();
            cfg.setEstimateInitialization('random', 1234);
            testCase.verifyEqual(cfg.controller.estimateInitialization.mode, 'random');
            testCase.verifyEqual(cfg.controller.estimateInitialization.spec, 1234);
        end

        function testSetEstimateInitializationFixedHigherWithExplicitSpec(testCase)
            cfg = vt.config.Config();
            theta = (21:30).';
            cfg.setEstimateInitialization('fixed-higher', theta);
            testCase.verifyEqual(cfg.controller.estimateInitialization.mode, 'fixed-higher');
            testCase.verifyEqual(cfg.controller.estimateInitialization.spec, theta);
        end
    end
end
