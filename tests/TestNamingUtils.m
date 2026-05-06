classdef TestNamingUtils < matlab.unittest.TestCase
    %TESTNAMINGUTILS Unit tests for compact naming helper behavior.

    methods (Test)
        function testTrajectoryLabelKnownNames(testCase)
            testCase.verifyEqual(vt.sim.NamingUtils.trajectoryLabel('circle'), 'circle');
            testCase.verifyEqual(vt.sim.NamingUtils.trajectoryLabel('infinity3d'), 'inf3d');
            testCase.verifyEqual(vt.sim.NamingUtils.trajectoryLabel('takeoffland'), 'tkoffland');
        end

        function testTrajectoryLabelFallbackIsSanitizedAndTrimmed(testCase)
            label = vt.sim.NamingUtils.trajectoryLabel('My Custom Trajectory 123 !!');
            testCase.verifyEqual(label, 'mycustomtraj');
        end

        function testControllerAndPotentialLabels(testCase)
            cfg = vt.config.Config();
            cfg.setController('Feedforward', 'liealgebra');
            testCase.verifyEqual(vt.sim.NamingUtils.controllerLabel(cfg), 'ff');
            testCase.verifyEqual(vt.sim.NamingUtils.potentialLabel(cfg), 'lie');
        end

        function testBatchTrajectoryLabelSingleAndMulti(testCase)
            cfg = vt.config.Config();
            cfg.setTrajectory('circle');
            testCase.verifyEqual(vt.sim.NamingUtils.batchTrajectoryLabel(cfg), 'circle');

            cfg.setTrajectory({'circle', 'infinity'});
            testCase.verifyEqual(vt.sim.NamingUtils.batchTrajectoryLabel(cfg), 'multi_traj');
        end

        function testRunLabelUsesBatchRunIndexWhenAvailable(testCase)
            saved = struct('cfgSnapshot', struct('sim', struct('batchRunIndex', 7)));
            label = vt.sim.NamingUtils.runLabel(saved, fullfile(tempdir, 'run_001'));
            testCase.verifyEqual(label, 'Run 7');
        end

        function testRunLabelFallsBackToFolderNameWhenIndexMissing(testCase)
            saved = struct('cfgSnapshot', struct('sim', struct()));
            label = vt.sim.NamingUtils.runLabel(saved, fullfile(tempdir, 'custom_folder'));
            testCase.verifyEqual(label, 'custom_folder');
        end
    end
end
