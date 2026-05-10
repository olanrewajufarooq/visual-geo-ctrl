classdef TestAdaptationInterface < matlab.unittest.TestCase
    %TESTADAPTATIONINTERFACE Test adaptation interface with new pi format.
    %   Tests setEstimatePi, update output structure, and diagnostics for
    %   both EuclideanAdaptation and BregmanDivAdaptation.

    methods (Test)
        function testSetEstimatePiConvertsParametersCorrectlyEuclidean(testCase)
            %TESTSETESTIMATEPICONVERTSPARAMETERSSCORRECTLYEUCLIDEAN
            %   Verify setEstimatePi correctly unpacks pi into m, CoG, Iparams.
            cfg = fth.sim.Config();
            cfg.setController('Feedforward');
            cfg.setAdaptation('euclidean');
            cfg.setAdaptiveGains(ones(10,1));
            cfg.done();

            adapt = fth.ctrl.adapt.EuclideanAdaptation(cfg);
            
            % Create a test pi vector: [m; h; Iparams_reordered]
            m_test = 15.5;
            h_test = [0.1; -0.05; 0.2];
            Iparams_legacy = [2; 3; 4; 0.1; 0.2; 0.3];
            pi_test = [m_test; h_test; Iparams_legacy([1; 2; 3; 4; 6; 5])];
            
            adapt.setEstimatePi(pi_test);
            [m_hat, cog_hat, I_hat] = adapt.getEstimate();
            
            testCase.verifyEqual(m_hat, m_test, 'AbsTol', 1e-10);
            testCase.verifyEqual(cog_hat, h_test / m_test, 'AbsTol', 1e-10);
            testCase.verifyEqual(I_hat, Iparams_legacy, 'AbsTol', 1e-10);
        end

        function testSetEstimatePiConvertsParametersCorrectlyBregman(testCase)
            %TESTSETESTIMATEPICONVERTSPARAMETERSCORRECTLYBREGMAN
            %   Verify BregmanDivAdaptation.setEstimatePi converts pi correctly.
            cfg = fth.sim.Config();
            cfg.setController('Feedforward');
            cfg.setAdaptation('bregman');
            cfg.setAdaptiveGains([0.1; zeros(9,1)]);
            cfg.done();

            adapt = fth.ctrl.adapt.BregmanDivAdaptation(cfg);
            
            % Create a test pi vector
            m_test = 12.0;
            h_test = [0.05; 0.1; -0.15];
            Iparams_legacy = [1.5; 2.5; 3.5; 0.05; 0.15; 0.1];
            pi_test = [m_test; h_test; Iparams_legacy([1; 2; 3; 4; 6; 5])];
            
            adapt.setEstimatePi(pi_test);
            [m_hat, cog_hat, I_hat] = adapt.getEstimate();
            
            testCase.verifyEqual(m_hat, m_test, 'AbsTol', 1e-10);
            testCase.verifyEqual(cog_hat, h_test / m_test, 'AbsTol', 1e-10);
            testCase.verifyEqual(I_hat, Iparams_legacy, 'AbsTol', 1e-10);
        end

        function testUpdateReturnsParamsStructWithRequiredFields(testCase)
            %TESTUPDATERETURNSPARAMSSTRUCTWITHREQUIREDFIELDS
            %   Verify update() returns struct with m, CoG, Iparams, I6.
            cfg = fth.sim.Config();
            cfg.setController('Feedforward');
            cfg.setAdaptation('euclidean');
            cfg.setAdaptiveGains(ones(10,1));
            cfg.done();

            adapt = fth.ctrl.adapt.EuclideanAdaptation(cfg);
            
            Hd = eye(4);
            H = eye(4);
            Vd = zeros(6,1);
            V = [0.1; 0.2; 0.3; 0.4; 0.5; 0.6];
            Ades = zeros(6,1);
            dt = 0.001;
            s = zeros(6,1);
            VR = zeros(6,1);
            VRDot = zeros(6,1);
            
            params = adapt.update(Hd, H, Vd, V, Ades, dt, s, VR, VRDot);
            
            testCase.verifyTrue(isstruct(params));
            testCase.verifyTrue(isfield(params, 'm'));
            testCase.verifyTrue(isfield(params, 'CoG'));
            testCase.verifyTrue(isfield(params, 'Iparams'));
            testCase.verifyTrue(isfield(params, 'I6'));
            testCase.verifyEqual(size(params.m), [1,1]);
            testCase.verifyEqual(size(params.CoG), [3,1]);
            testCase.verifyEqual(size(params.Iparams), [6,1]);
            testCase.verifyEqual(size(params.I6), [6,6]);
        end

        function testDiagnosticsStructFormatEuclidean(testCase)
            %TESTDIAGNOSTICSSTRUCTFORMATEUCLID EAN
            %   Verify getDiagnostics returns expected struct for Euclidean.
            cfg = fth.sim.Config();
            cfg.setController('Feedforward');
            cfg.setAdaptation('euclidean');
            cfg.setAdaptiveGains([1; 0.5; zeros(8,1)]);
            cfg.done();

            adapt = fth.ctrl.adapt.EuclideanAdaptation(cfg);
            diagnostics = adapt.getDiagnostics();
            
            testCase.verifyTrue(isstruct(diagnostics));
            testCase.verifyTrue(isfield(diagnostics, 'infoMatrix'));
            testCase.verifyTrue(isfield(diagnostics, 'updateCount'));
            testCase.verifyEqual(size(diagnostics.infoMatrix), [10, 10]);
            testCase.verifyEqual(diagnostics.updateCount, 0);
        end

        function testDiagnosticsStructFormatBregman(testCase)
            %TESTDIAGNOSTICSSTRUCTFORMATBREGMAN
            %   Verify getDiagnostics returns expected struct for Bregman.
            cfg = fth.sim.Config();
            cfg.setController('Feedforward');
            cfg.setAdaptation('bregman');
            cfg.setAdaptiveGains([0.05; zeros(9,1)]);
            cfg.done();

            adapt = fth.ctrl.adapt.BregmanDivAdaptation(cfg);
            diagnostics = adapt.getDiagnostics();
            
            testCase.verifyTrue(isstruct(diagnostics));
            testCase.verifyTrue(isfield(diagnostics, 'J_hat'));
            testCase.verifyTrue(isfield(diagnostics, 'is_spd'));
            testCase.verifyTrue(isfield(diagnostics, 'updateCount'));
            testCase.verifyEqual(size(diagnostics.J_hat), [4, 4]);
            testCase.verifyEqual(diagnostics.updateCount, 0);
            testCase.verifyTrue(diagnostics.is_spd);
        end

        function testGetParamsReturnsConsistentValues(testCase)
            %TESTGETPARAMSRETURNSCONSISTENTVALUES
            %   Verify getParams and getEstimate return consistent data.
            cfg = fth.sim.Config();
            cfg.setController('Feedforward');
            cfg.setAdaptation('euclidean');
            cfg.setAdaptiveGains(ones(10,1));
            cfg.done();

            adapt = fth.ctrl.adapt.EuclideanAdaptation(cfg);
            params = adapt.getParams();
            [m_hat, cog_hat, Iparams_hat] = adapt.getEstimate();
            
            testCase.verifyEqual(params.m, m_hat);
            testCase.verifyEqual(params.CoG, cog_hat);
            testCase.verifyEqual(params.Iparams, Iparams_hat);
        end
    end
end
