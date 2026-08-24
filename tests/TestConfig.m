classdef TestConfig < matlab.unittest.TestCase
    %TESTCONFIG Unit tests for the Config fluent API and validation logic.

    methods (Test)
        function testDefaultConstruction(testCase)
            cfg = fth.sim.Config();
            testCase.verifyEqual(cfg.traj.name, 'hover');
            testCase.verifyEqual(cfg.controller.type, 'PD');
            testCase.verifyEqual(cfg.controller.adaptation, 'none');
            testCase.verifyEqual(cfg.vehicle.g, 9.8);
            testCase.verifyTrue(cfg.vehicle.m > 0);
        end

        function testSetTrajectoryValid(testCase)
            cfg = fth.sim.Config();
            cfg.setTrajectory('circle');
            testCase.verifyEqual(cfg.traj.name, 'circle');
        end

        function testSetTrajectoryWithScalarHoverOverride(testCase)
            cfg = fth.sim.Config();
            cfg.setTrajectory({'circle', 'infinity'}, false);
            testCase.verifyFalse(cfg.traj.goToHoverBeforePathStarts);
            testCase.verifyEqual(cfg.traj.batch.goToHoverBeforePathStarts, [false false]);
        end

        function testSetTrajectoryWithVectorHoverOverride(testCase)
            cfg = fth.sim.Config();
            cfg.setTrajectory({'circle', 'takeoffland'}, [true false]);
            cfg.setController('PD');
            cfg.done();
            testCase.verifyEqual(cfg.traj.batch.goToHoverBeforePathStarts, [true false]);
            cfgs = cfg.expandBatchConfigs(tempname);
            testCase.verifyEqual(numel(cfgs), 2);
            testCase.verifyTrue(cfgs{1}.traj.goToHoverBeforePathStarts);
            testCase.verifyFalse(cfgs{2}.traj.goToHoverBeforePathStarts);
        end

        function testSetTrajectoryWithoutHoverOverridePreservesDefaults(testCase)
            cfg = fth.sim.Config();
            cfg.setTrajectory({'circle', 'takeoffland'});
            cfg.setController('PD');
            cfg.done();
            cfgs = cfg.expandBatchConfigs(tempname);
            testCase.verifyTrue(cfgs{1}.traj.goToHoverBeforePathStarts);
            testCase.verifyFalse(cfgs{2}.traj.goToHoverBeforePathStarts);
        end

        function testTrajectoryHoverLengthMismatchThrows(testCase)
            cfg = fth.sim.Config();
            testCase.verifyError(@() cfg.setTrajectory({'circle', 'infinity'}, [true false true]), ...
                'Config:InvalidTrajectoryHover');
        end

        function testSetControllerPD(testCase)
            cfg = fth.sim.Config();
            cfg.setController('PD');
            testCase.verifyEqual(lower(cfg.controller.type), 'pd');
        end

        function testSetControllerFeedLin(testCase)
            cfg = fth.sim.Config();
            cfg.setController('FeedLin');
            testCase.verifyEqual(lower(cfg.controller.type), 'feedlin');
        end

        function testSetControllerFeedforward(testCase)
            cfg = fth.sim.Config();
            cfg.setController('Feedforward');
            testCase.verifyEqual(lower(cfg.controller.type), 'feedforward');
        end

        function testSetControllerStoresType(testCase)
            cfg = fth.sim.Config();
            cfg.setController('MyCustomMode');
            testCase.verifyEqual(cfg.controller.type, 'MyCustomMode');
        end

        function testSetPotentialType(testCase)
            cfg = fth.sim.Config();
            cfg.setController('PD', 'inertia-gain');
            testCase.verifyEqual(cfg.controller.potential, 'inertia-gain');
        end

        function testDefaultPotentialIsLog(testCase)
            cfg = fth.sim.Config();
            cfg.setController('PD');
            testCase.verifyEqual(cfg.controller.potential, 'log');
        end

        function testSetAdaptationEuclidean(testCase)
            cfg = fth.sim.Config();
            cfg.setAdaptation('euclidean');
            testCase.verifyEqual(cfg.controller.adaptation, 'euclidean');
            testCase.verifyTrue(~isempty(cfg.controller.Gamma));
        end

        function testSetAdaptationNone(testCase)
            cfg = fth.sim.Config();
            cfg.setAdaptation('none');
            testCase.verifyEqual(cfg.controller.adaptation, 'none');
        end

        function testSetSimParams(testCase)
            cfg = fth.sim.Config();
            cfg.setSimParams(0.01, 20);
            testCase.verifyEqual(cfg.sim.dt, 0.01);
            testCase.verifyEqual(cfg.sim.duration, 20);
        end

        function testEnableSafetyDefaultsTrue(testCase)
            cfg = fth.sim.Config();
            testCase.verifyTrue(cfg.sim.enableSafety);
        end

        function testEnableSafetyCanBeConfigured(testCase)
            cfg = fth.sim.Config();
            cfg.useSimOptions(struct('enableSafety', false));
            testCase.verifyFalse(cfg.sim.enableSafety);
        end

        function testSetKpGainsVector(testCase)
            cfg = fth.sim.Config();
            Kp = [1; 2; 3; 4; 5; 6];
            cfg.setKpGains(Kp);
            testCase.verifyEqual(cfg.controller.Kp, Kp);
        end

        function testSetKpGainsRowVector(testCase)
            cfg = fth.sim.Config();
            Kp = [1, 2, 3, 4, 5, 6];
            cfg.setKpGains(Kp);
            testCase.verifyEqual(cfg.controller.Kp, Kp(:));
        end

        function testSetKpGainsMatrix(testCase)
            cfg = fth.sim.Config();
            Kp = [1 2 3 4 5 6; 7 8 9 10 11 12];
            cfg.setKpGains(Kp);
            testCase.verifyEqual(cfg.controller.Kp, Kp);
        end

        function testSetKdGainsVector(testCase)
            cfg = fth.sim.Config();
            Kd = [1; 2; 3; 4; 5; 6];
            cfg.setKdGains(Kd);
            testCase.verifyEqual(cfg.controller.Kd, Kd);
        end

        function testSetAdaptiveGainsVector(testCase)
            cfg = fth.sim.Config();
            Gamma = ones(10, 1);
            cfg.setAdaptiveGains(Gamma);
            testCase.verifyEqual(cfg.controller.Gamma, Gamma);
        end

        function testSetAdaptiveGainsMatrix(testCase)
            cfg = fth.sim.Config();
            Gamma = [ones(1, 10); 2*ones(1, 10)];
            cfg.setAdaptiveGains(Gamma);
            testCase.verifyEqual(cfg.controller.Gamma, Gamma);
        end

        function testSetAdaptiveGainsScalar(testCase)
            cfg = fth.sim.Config();
            cfg.setAdaptiveGains(0.25);
            testCase.verifyEqual(cfg.controller.Gamma, 0.25);
        end

        function testBregmanDefaultGammaIsScalar(testCase)
            cfg = fth.sim.Config();
            cfg.setController('Feedforward');
            cfg.setAdaptation('bregman');
            cfg.done();
            testCase.verifyTrue(isscalar(cfg.controller.Gamma));
            testCase.verifyGreaterThan(cfg.controller.Gamma, 0);
        end

        function testBregmanRejectsTenVectorGamma(testCase)
            cfg = fth.sim.Config();
            cfg.setController('Feedforward');
            cfg.setAdaptation('bregman');
            cfg.setAdaptiveGains(ones(10, 1));
            testCase.verifyError(@() cfg.done(), 'Config:InvalidGainShape');
        end

        function testBregmanGammaBatchCount(testCase)
            cfg = fth.sim.Config();
            cfg.setController('Feedforward');
            cfg.setAdaptation('bregman');
            cfg.setAdaptiveGains([0.1; 0.2]);
            cfg.done();
            testCase.verifyEqual(cfg.getBatchCount(), 2);
        end

        function testAdaptationBackTrackingOption(testCase)
            cfg = fth.sim.Config();
            testCase.verifyFalse(cfg.controller.useBackTracking);
            cfg.useAdaptationOptions(struct('type', 'bregman', ...
                'useBackTracking', true));
            cfg.done();
            testCase.verifyTrue(cfg.controller.useBackTracking);
        end

        function testAdaptationBackTrackingBatchCount(testCase)
            cfg = fth.sim.Config();
            cfg.useAdaptationOptions(struct( ...
                'type', {{'bregman', 'bregman'}}, ...
                'Gamma', {{0.001, 0.002}}, ...
                'useBackTracking', [true; false]));
            cfg.done();
            testCase.verifyEqual(cfg.getBatchCount(), 2);
            cfgs = cfg.expandBatchConfigs();
            testCase.verifyTrue(cfgs{1}.controller.useBackTracking);
            testCase.verifyFalse(cfgs{2}.controller.useBackTracking);
        end

        function testFluentChaining(testCase)
            cfg = fth.sim.Config();
            cfg = cfg.setTrajectory('circle') ...
                     .setController('Feedforward') ...
                     .setSimParams(0.005, 30);
            testCase.verifyEqual(cfg.traj.name, 'circle');
            testCase.verifyEqual(lower(cfg.controller.type), 'feedforward');
            testCase.verifyEqual(cfg.sim.duration, 30);
        end

        function testDoneNormalizesDefaults(testCase)
            cfg = fth.sim.Config();
            cfg.setTrajectory('circle');
            cfg.setController('PD');
            cfg.done();
            testCase.verifyTrue(~isempty(cfg.controller.Kp));
            testCase.verifyTrue(~isempty(cfg.controller.Kd));
            testCase.verifyEqual(numel(cfg.controller.Kp), 6);
            testCase.verifyEqual(numel(cfg.controller.Kd), 6);
        end

        function testDoneAdaptiveGetsGamma(testCase)
            cfg = fth.sim.Config();
            cfg.setTrajectory('circle');
            cfg.setController('Feedforward');
            cfg.setAdaptation('euclidean');
            cfg.done();
            testCase.verifyTrue(~isempty(cfg.controller.Gamma));
            testCase.verifyEqual(numel(cfg.controller.Gamma), 10);
        end

        function testCopyIsIndependent(testCase)
            cfg = fth.sim.Config();
            cfg.setTrajectory('circle');
            cfgCopy = cfg.copy();
            cfgCopy.setTrajectory('hover');
            testCase.verifyEqual(cfg.traj.name, 'circle');
            testCase.verifyEqual(cfgCopy.traj.name, 'hover');
        end

        function testGetBatchCountSingle(testCase)
            cfg = fth.sim.Config();
            cfg.setTrajectory('circle');
            cfg.setController('PD');
            cfg.done();
            testCase.verifyEqual(cfg.getBatchCount(), 1);
        end

        function testGetBatchCountMultiTrajectory(testCase)
            cfg = fth.sim.Config();
            cfg.setTrajectory({'circle', 'infinity'});
            cfg.setController('PD');
            cfg.done();
            testCase.verifyEqual(cfg.getBatchCount(), 2);
        end

        function testGetBatchCountMultiGain(testCase)
            cfg = fth.sim.Config();
            cfg.setTrajectory('circle');
            cfg.setController('PD');
            cfg.setKpGains([1 2 3 4 5 6; 7 8 9 10 11 12]);
            cfg.setKdGains([1 2 3 4 5 6; 7 8 9 10 11 12]);
            cfg.done();
            testCase.verifyEqual(cfg.getBatchCount(), 2);
        end

        function testInconsistentBatchCountsThrow(testCase)
            cfg = fth.sim.Config();
            cfg.setTrajectory('circle');
            cfg.setController('PD');
            cfg.setKpGains([1 2 3 4 5 6; 7 8 9 10 11 12]);  % 2 runs
            cfg.setKdGains([1 2 3 4 5 6; 7 8 9 10 11 12; 1 1 1 1 1 1]);  % 3 runs
            testCase.verifyError(@() cfg.getBatchCount(), ...
                'Config:InconsistentBatchCounts');
        end

        function testSetPayloadScenario(testCase)
            cfg = fth.sim.Config();
            cfg.setPayloadScenario(0.5, [0.01; 0; -0.02], 15);
            testCase.verifyEqual(cfg.payload.mass, 0.5);
            testCase.verifyEqual(cfg.payload.position, [0.01; 0; -0.02]);
            testCase.verifyEqual(cfg.payload.dropTime, 15);
        end

        function testDeprecatedSetPayloadWithInitFlagThrows(testCase)
            cfg = fth.sim.Config();
            testCase.verifyError(@() cfg.setPayload(0.5, [0; 0; 0], 10, false), ...
                'Config:DeprecatedSetPayload');
        end

        function testSetParamInitVehicle(testCase)
            cfg = fth.sim.Config();
            cfg.setParamInit('vehicle');
            testCase.verifyEqual(cfg.controller.paramInit.mode, 'vehicle');
        end

        function testSetParamInitVehiclePlusPayloadHigher(testCase)
            cfg = fth.sim.Config();
            cfg.setParamInit('vehicle-plus-payload-higher');
            testCase.verifyEqual(cfg.controller.paramInit.mode, 'vehicle-plus-payload-higher');
            testCase.verifyEmpty(cfg.controller.paramInit.spec);
        end

        function testSetParamInitCustomVector(testCase)
            cfg = fth.sim.Config();
            theta = (1:10)';
            cfg.setParamInit(theta);
            testCase.verifyEqual(cfg.controller.paramInit.mode, 'custom');
            testCase.verifyEqual(cfg.controller.paramInit.spec, theta);
        end

        function testSetParamInitCustomRowVector(testCase)
            cfg = fth.sim.Config();
            theta = 1:10;
            cfg.setParamInit(theta);
            testCase.verifyEqual(cfg.controller.paramInit.spec, theta(:));
        end

        function testSetParamInitMidVehiclePayloadWithExplicitSpec(testCase)
            cfg = fth.sim.Config();
            theta = (11:20)';
            cfg.setParamInit('mid-vehicle-payload', theta);
            testCase.verifyEqual(cfg.controller.paramInit.mode, 'mid-vehicle-payload');
            testCase.verifyEqual(cfg.controller.paramInit.spec, theta);
        end

        function testSetParamInitBadLengthThrows(testCase)
            cfg = fth.sim.Config();
            testCase.verifyError(@() cfg.setParamInit([1 2 3]), 'MATLAB:incorrectNumel');
        end

        function testSetParamInitInvalidModeThrows(testCase)
            cfg = fth.sim.Config();
            testCase.verifyError(@() cfg.setParamInit('badmode'), ...
                'Config:InvalidParamInitMode');
        end

        function testExpandBatchConfigsTrajectoryMajorOrderingAndOverrides(testCase)
            cfg = fth.sim.Config();
            cfg.setTrajectory({'circle', 'takeoffland'}, [true false]);
            cfg.setController('PD');
            cfg.setKpGains([1 2 3 4 5 6; 7 8 9 10 11 12]);
            cfg.setKdGains([1 2 3 4 5 6; 7 8 9 10 11 12]);
            cfg.done();
            rootDir = fullfile(tempdir, ['cfg_batch_' char(matlab.lang.internal.uuid())]);
            cfgs = cfg.expandBatchConfigs(rootDir);

            testCase.verifyEqual(numel(cfgs), 4);
            testCase.verifyEqual(cfgs{1}.traj.name, 'circle');
            testCase.verifyEqual(cfgs{2}.traj.name, 'circle');
            testCase.verifyEqual(cfgs{3}.traj.name, 'takeoffland');
            testCase.verifyEqual(cfgs{4}.traj.name, 'takeoffland');
            testCase.verifyTrue(cfgs{1}.traj.goToHoverBeforePathStarts);
            testCase.verifyFalse(cfgs{3}.traj.goToHoverBeforePathStarts);
            testCase.verifyEqual(cfgs{1}.sim.batchRunIndex, 1);
            testCase.verifyEqual(cfgs{2}.sim.batchRunIndex, 2);
            testCase.verifyEqual(cfgs{3}.sim.globalBatchIndex, 3);
            testCase.verifyTrue(contains(cfgs{1}.sim.resultsDirOverride, fullfile('t01_circle', 'run_001')));
            testCase.verifyTrue(contains(cfgs{4}.sim.resultsDirOverride, fullfile('t02_tkoffland', 'run_002')));
        end

        function testSetPlotLayoutValid(testCase)
            cfg = fth.sim.Config();
            cfg.setPlotLayout('row-major');
            testCase.verifyEqual(cfg.viz.plotLayout, 'row-major');
        end

        function testSetControlParams(testCase)
            cfg = fth.sim.Config();
            cfg.setControlParams(0.01);
            testCase.verifyEqual(cfg.sim.control_dt, 0.01);
        end

        function testSetAdaptationParams(testCase)
            cfg = fth.sim.Config();
            cfg.setAdaptationParams(0.002);
            testCase.verifyEqual(cfg.sim.adaptation_dt, 0.002);
            testCase.verifyFalse(cfg.sim.adaptation_dt_auto);
        end

        function testVehicleI6Computed(testCase)
            cfg = fth.sim.Config();
            testCase.verifyTrue(isfield(cfg.vehicle, 'I6'));
            testCase.verifyEqual(size(cfg.vehicle.I6), [6, 6]);
            % I6 should be symmetric positive-definite
            testCase.verifyEqual(cfg.vehicle.I6, cfg.vehicle.I6', 'AbsTol', 1e-14);
            testCase.verifyTrue(all(eig(cfg.vehicle.I6) > 0));
        end

        function testVisualizationDefaults(testCase)
            cfg = fth.sim.Config();
            testCase.verifyFalse(cfg.viz.enable);
            testCase.verifyTrue(cfg.viz.dynamicAxis);
            testCase.verifyEqual(cfg.viz.plotLayout, 'column-major');
        end
    end
end
