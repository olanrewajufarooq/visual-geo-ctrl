classdef TestReplayTrajectory < matlab.unittest.TestCase
    %TESTREPLAYTRAJECTORY Coverage for processed replay trajectories.

    methods (Test)
        function testConfigAcceptsReplayTrajectory(testCase)
            cfg = fth.sim.Config();
            cfg.setTrajectory('replay');
            cfg.traj.replay.id = 'demo-replay';
            cfg.done();

            testCase.verifyEqual(cfg.traj.name, 'replay');
            testCase.verifyEqual(cfg.traj.replay.id, 'demo-replay');
        end

        function testFactoryCreatesReplayTrajectory(testCase)
            tmpRoot = testCase.createTempStructure();
            cleanup = onCleanup(@() rmdir(tmpRoot, 's'));
            %#ok<NASGU>

            manifestPath = testCase.writeManifest(tmpRoot, struct( ...
                'demo_replay', struct( ...
                    'source_mode', 'autonomous', ...
                    'source_file', 'autonomous/demo/demo_500hz_freq_sync.csv', ...
                    'artifact_file', 'demo_replay.mat', ...
                    'label', 'Demo Replay')));
            testCase.writeProcessedArtifact(fullfile(tmpRoot, 'demo_replay.mat'));

            cfg = fth.sim.Config();
            cfg.setTrajectory('replay');
            cfg.traj.replay = struct( ...
                'id', 'demo_replay', ...
                'rootDir', tmpRoot, ...
                'manifestFile', manifestPath);

            traj = fth.traj.TrajectoryFactory.create(cfg);
            testCase.verifyClass(traj, 'fth.traj.ReplayTraj');
        end

        function testProcessorBuildsProcessedReplayArtifact(testCase)
            tmpRoot = testCase.createTempStructure();
            cleanup = onCleanup(@() rmdir(tmpRoot, 's'));
            %#ok<NASGU>

            rawDir = fullfile(tmpRoot, 'autonomous', 'demo');
            if ~exist(rawDir, 'dir')
                mkdir(rawDir);
            end
            rawFile = fullfile(rawDir, 'demo_500hz_freq_sync.csv');
            testCase.writeRawTrajectoryCsv(rawFile);

            manifestPath = testCase.writeManifest(tmpRoot, struct( ...
                'demo_replay', struct( ...
                    'source_mode', 'autonomous', ...
                    'source_file', 'autonomous/demo/demo_500hz_freq_sync.csv', ...
                    'artifact_file', 'demo_replay.mat', ...
                    'label', 'Demo Replay')));

            summary = fth.traj.ReplayProcessor.processAll(tmpRoot, manifestPath);
            artifactPath = fullfile(tmpRoot, 'demo_replay.mat');
            data = load(artifactPath, 'traj');

            testCase.verifyEqual(summary.processedCount, 1);
            testCase.verifyTrue(isfile(artifactPath));
            testCase.verifyEqual(data.traj.meta.id, 'demo_replay');
            testCase.verifyEqual(size(data.traj.p), [3 3]);
            testCase.verifyEqual(size(data.traj.v_b), [3 3]);
            testCase.verifyEqual(size(data.traj.omega_b), [3 3]);
        end

        function testProcessorConvertsWorldSignalsToBodyFrame(testCase)
            tmpRoot = testCase.createTempStructure();
            cleanup = onCleanup(@() rmdir(tmpRoot, 's'));
            %#ok<NASGU>
            rawDir = fullfile(tmpRoot, 'autonomous', 'demo');
            mkdir(rawDir);
            rawFile = fullfile(rawDir, 'demo_500hz_freq_sync.csv');
            testCase.writeRawTrajectoryCsv(rawFile);
            manifestPath = testCase.writeManifest(tmpRoot, struct( ...
                'demo_replay', struct('source_mode', 'autonomous', ...
                'source_file', 'autonomous/demo/demo_500hz_freq_sync.csv', ...
                'artifact_file', 'demo_replay.mat')));

            fth.traj.ReplayProcessor.processAll(tmpRoot, manifestPath);
            data = load(fullfile(tmpRoot, 'demo_replay.mat'), 'traj');
            testCase.verifyEqual(data.traj.v_b(2, :), [-1.2 1.1 1.3], 'AbsTol', 1e-12);
            testCase.verifyEqual(data.traj.omega_b(2, :), [-0.5 0.4 0.6], 'AbsTol', 1e-12);
        end

        function testReplayTrajectoryReturnsProcessedState(testCase)
            tmpRoot = testCase.createTempStructure();
            cleanup = onCleanup(@() rmdir(tmpRoot, 's'));
            %#ok<NASGU>

            manifestPath = testCase.writeManifest(tmpRoot, struct( ...
                'demo_replay', struct( ...
                    'source_mode', 'autonomous', ...
                    'source_file', 'autonomous/demo/demo_500hz_freq_sync.csv', ...
                    'artifact_file', 'demo_replay.mat', ...
                    'label', 'Demo Replay')));
            testCase.writeProcessedArtifact(fullfile(tmpRoot, 'demo_replay.mat'));

            cfg = fth.sim.Config();
            cfg.setTrajectory('replay');
            cfg.traj.replay = struct( ...
                'id', 'demo_replay', ...
                'rootDir', tmpRoot, ...
                'manifestFile', manifestPath);

            traj = fth.traj.ReplayTraj(cfg);
            [Hd, Vd, Ad] = traj.generate(0.5, eye(4), zeros(6,1), struct());

            testCase.verifyEqual(Hd(1:3,4), [1; 2; 3], 'AbsTol', 1e-12);
            testCase.verifyEqual(Vd, [0.4; 0.5; 0.6; 1.1; 1.2; 1.3], 'AbsTol', 1e-12);
            testCase.verifyEqual(Ad, [0.04; 0.05; 0.06; 0.11; 0.12; 0.13], 'AbsTol', 1e-12);
        end
    end

    methods (Static, Access = private)
        function root = createTempStructure()
            root = fullfile(tempdir, ['replay_traj_' char(matlab.lang.internal.uuid())]);
            mkdir(root);
        end

        function manifestPath = writeManifest(rootDir, manifestStruct)
            manifestPath = fullfile(rootDir, 'manifest.json');
            fid = fopen(manifestPath, 'w');
            assert(fid ~= -1, 'Failed to open manifest for writing.');
            fwrite(fid, jsonencode(manifestStruct), 'char');
            fclose(fid);
        end

        function writeProcessedArtifact(path)
            traj = struct();
            traj.t = [0; 0.5; 1.0];
            traj.p = [0 0 0; 1 2 3; 2 4 6];
            traj.v_b = [0.1 0.2 0.3; 1.1 1.2 1.3; 2.1 2.2 2.3];
            traj.a_b = [0.01 0.02 0.03; 0.11 0.12 0.13; 0.21 0.22 0.23];
            traj.omega_b = [0.1 0.2 0.3; 0.4 0.5 0.6; 0.7 0.8 0.9];
            traj.alpha_b = [0.01 0.02 0.03; 0.04 0.05 0.06; 0.07 0.08 0.09];
            traj.R = repmat(eye(3), 1, 1, 3);
            traj.meta = struct('id', 'demo_replay', 'sampleRateHz', 2);
            save(path, 'traj');
        end

        function writeRawTrajectoryCsv(path)
            headers = [ ...
                "elapsed_time", "timestamp", "img_filename", ...
                "drone_x", "drone_y", "drone_z", ...
                "drone_roll", "drone_pitch", "drone_yaw", ...
                "drone_velocity_linear_x", "drone_velocity_linear_y", "drone_velocity_linear_z", ...
                "drone_velocity_angular_x", "drone_velocity_angular_y", "drone_velocity_angular_z", ...
                "drone_rot[0]", "drone_rot[1]", "drone_rot[2]", ...
                "drone_rot[3]", "drone_rot[4]", "drone_rot[5]", ...
                "drone_rot[6]", "drone_rot[7]", "drone_rot[8]"];
            rows = {
                0.0, 1000, 'img0.jpg', 0, 0, 0, 0, 0, 0, 0.1, 0.2, 0.3, 0.01, 0.02, 0.03, 1, 0, 0, 0, 1, 0, 0, 0, 1;
                0.5, 1500, 'img1.jpg', 1, 2, 3, 0, 0, 0, 1.1, 1.2, 1.3, 0.4, 0.5, 0.6, 0, 1, 0, -1, 0, 0, 0, 0, 1;
                1.0, 2000, 'img2.jpg', 2, 4, 6, 0, 0, 0, 2.1, 2.2, 2.3, 0.7, 0.8, 0.9, 1, 0, 0, 0, 1, 0, 0, 0, 1
            };
            T = cell2table(rows, 'VariableNames', cellstr(headers));
            writetable(T, path);
        end
    end
end
