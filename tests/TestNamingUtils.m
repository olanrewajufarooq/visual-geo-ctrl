classdef TestNamingUtils < matlab.unittest.TestCase
    %TESTNAMINGUTILS Unit tests for compact naming helper behavior.

    methods (Test)
        function testTrajectoryLabelKnownNames(testCase)
            testCase.verifyEqual(fth.sim.NamingUtils.trajectoryLabel('circle'), 'circle');
            testCase.verifyEqual(fth.sim.NamingUtils.trajectoryLabel('infinity3d'), 'inf3d');
            testCase.verifyEqual(fth.sim.NamingUtils.trajectoryLabel('takeoffland'), 'tkoffland');
        end

        function testTrajectoryLabelFallbackIsSanitizedAndTrimmed(testCase)
            label = fth.sim.NamingUtils.trajectoryLabel('My Custom Trajectory 123 !!');
            testCase.verifyEqual(label, 'mycustomtraj');
        end

        function testControllerAndPotentialLabels(testCase)
            cfg = fth.config.Config();
            cfg.setController('Feedforward', 'liealgebra');
            testCase.verifyEqual(fth.sim.NamingUtils.controllerLabel(cfg), 'ff');
            testCase.verifyEqual(fth.sim.NamingUtils.potentialLabel(cfg), 'lie');
        end

        function testBatchTrajectoryLabelSingleAndMulti(testCase)
            cfg = fth.config.Config();
            cfg.setTrajectory('circle');
            testCase.verifyEqual(fth.sim.NamingUtils.batchTrajectoryLabel(cfg), 'circle');

            cfg.setTrajectory({'circle', 'infinity'});
            testCase.verifyEqual(fth.sim.NamingUtils.batchTrajectoryLabel(cfg), 'multi_traj');
        end

        function testRunLabelUsesBatchRunIndexWhenAvailable(testCase)
            saved = struct('cfgSnapshot', struct('sim', struct('batchRunIndex', 7)));
            label = fth.sim.NamingUtils.runLabel(saved, fullfile(tempdir, 'run_001'));
            testCase.verifyEqual(label, 'Run 7');
        end

        function testRunLabelFallsBackToFolderNameWhenIndexMissing(testCase)
            saved = struct('cfgSnapshot', struct('sim', struct()));
            label = fth.sim.NamingUtils.runLabel(saved, fullfile(tempdir, 'custom_folder'));
            testCase.verifyEqual(label, 'custom_folder');
        end
    end
end
