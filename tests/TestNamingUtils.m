classdef TestNamingUtils < matlab.unittest.TestCase
    %TESTNAMINGUTILS Unit tests for compact naming helper behavior.

    methods (Test)
        function testTrajectoryLabelKnownNames(testCase)
            testCase.verifyEqual(fth.io.NamingUtils.trajectoryLabel('circle'), 'circle');
            testCase.verifyEqual(fth.io.NamingUtils.trajectoryLabel('infinity3d'), 'infinity3d');
            testCase.verifyEqual(fth.io.NamingUtils.trajectoryLabel('takeoffland'), 'tkoffland');
        end

        function testTrajectoryLabelFallbackIsSanitizedAndTrimmed(testCase)
            label = fth.io.NamingUtils.trajectoryLabel('My Custom Trajectory 123 !!');
            testCase.verifyEqual(label, 'mycustomtraj');
        end

        function testControllerAndPotentialLabels(testCase)
            cfg = fth.sim.Config();
            cfg.setController('Feedforward', 'log');
            testCase.verifyEqual(fth.io.NamingUtils.controllerLabel(cfg), 'ff');
            testCase.verifyEqual(fth.io.NamingUtils.potentialLabel(cfg), 'log');
        end

        function testBatchTrajectoryLabelSingleAndMulti(testCase)
            cfg = fth.sim.Config();
            cfg.setTrajectory('circle');
            testCase.verifyEqual(fth.io.NamingUtils.batchTrajectoryLabel(cfg), 'circle');

            cfg.setTrajectory({'circle', 'infinity'});
            testCase.verifyEqual(fth.io.NamingUtils.batchTrajectoryLabel(cfg), 'multi_traj');
        end

        function testRunLabelUsesBatchRunIndexWhenAvailable(testCase)
            saved = struct('cfgSnapshot', struct('sim', struct('batchRunIndex', 7)));
            label = fth.io.NamingUtils.runLabel(saved, fullfile(tempdir, 'run_001'));
            testCase.verifyEqual(label, 'Run 7');
        end

        function testRunLabelFallsBackToFolderNameWhenIndexMissing(testCase)
            saved = struct('cfgSnapshot', struct('sim', struct()));
            label = fth.io.NamingUtils.runLabel(saved, fullfile(tempdir, 'custom_folder'));
            testCase.verifyEqual(label, 'custom_folder');
        end
    end
end
