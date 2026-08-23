classdef ReplayProcessor
    %REPLAYPROCESSOR Build and load canonical replay trajectory artifacts.

    methods (Static)
        function summary = processAll(rootDir, manifestPath)
            %PROCESSALL Convert manifest-listed raw CSV files into .mat artifacts.
            if nargin < 1 || isempty(rootDir)
                rootDir = ReplayProcessor.defaultRootDir();
            end
            if nargin < 2 || isempty(manifestPath)
                manifestPath = fullfile(rootDir, 'manifest.json');
            end

            manifest = ReplayProcessor.loadManifest(manifestPath);
            ids = fieldnames(manifest);
            fprintf('[replay] Processing %d trajectories from %s\n', numel(ids), manifestPath);
            for i = 1:numel(ids)
                id = ids{i};
                entry = manifest.(id);
                rawPath = ReplayProcessor.resolveSourcePath(rootDir, entry.source_file);
                outPath = fullfile(rootDir, char(string(entry.artifact_file)));
                fprintf('[replay %d/%d] %s\n', i, numel(ids), id);
                fprintf('  source: %s\n', rawPath);
                fileTimer = tic;
                traj = ReplayProcessor.processSingle(rawPath, id, entry);
                outDir = fileparts(outPath);
                if ~isempty(outDir) && ~exist(outDir, 'dir')
                    mkdir(outDir);
                end
                save(outPath, 'traj');
                fprintf('  artifact: %s (%.1f s)\n', outPath, toc(fileTimer));
            end

            summary = struct('processedCount', numel(ids), 'manifestPath', manifestPath);
            fprintf('[replay] Completed %d trajectories.\n', summary.processedCount);
        end

        function entry = loadEntry(replayCfg)
            %LOADENTRY Resolve one replay manifest entry from config.
            [rootDir, manifestPath, replayId] = ReplayProcessor.resolveReplayConfig(replayCfg);
            manifest = ReplayProcessor.loadManifest(manifestPath);
            if ~isfield(manifest, replayId)
                error('fth:Replay:UnknownId', ...
                    'Replay id ''%s'' was not found in manifest ''%s''.', replayId, manifestPath);
            end

            entry = manifest.(replayId);
            entry.id = replayId;
            entry.rootDir = rootDir;
            entry.manifestPath = manifestPath;
            entry.artifactPath = fullfile(rootDir, char(string(entry.artifact_file)));
        end

        function traj = loadArtifact(replayCfg)
            %LOADARTIFACT Load a processed replay artifact from config.
            entry = ReplayProcessor.loadEntry(replayCfg);
            if ~isfile(entry.artifactPath)
                error('fth:Replay:ArtifactMissing', ...
                    'Replay artifact ''%s'' does not exist.', entry.artifactPath);
            end
            data = load(entry.artifactPath, 'traj');
            traj = data.traj;
        end

        function limits = defaultAxisLimits(cfg, pad)
            %DEFAULTAXISLIMITS Compute visualization limits for replay trajectories.
            if nargin < 2 || isempty(pad)
                pad = 2.0;
            end
            try
                traj = ReplayProcessor.loadArtifact(cfg.traj.replay);
                p = traj.p;
                mins = min(p, [], 1) - pad;
                maxs = max(p, [], 1) + pad;
                zMin = min(mins(3), 0);
                zMax = max(maxs(3), 0);
                limits = [mins(1) maxs(1) mins(2) maxs(2) zMin zMax];
            catch
                limits = [-5 5 -5 5 0 5];
            end
        end

        function rootDir = defaultRootDir()
            %DEFAULTROOTDIR Return the default processed trajectory root.
            thisFile = mfilename('fullpath');
            trajectoriesRoot = fileparts(fileparts(thisFile));
            rootDir = fullfile(trajectoriesRoot, 'processed');
        end
    end

    methods (Static, Access = private)
        function [rootDir, manifestPath, replayId] = resolveReplayConfig(replayCfg)
            if ~isstruct(replayCfg)
                error('fth:Replay:InvalidConfig', 'cfg.traj.replay must be a struct.');
            end
            if ~isfield(replayCfg, 'id') || isempty(replayCfg.id)
                error('fth:Replay:MissingId', 'cfg.traj.replay.id must be set for replay trajectories.');
            end
            replayId = char(string(replayCfg.id));
            if isfield(replayCfg, 'rootDir') && ~isempty(replayCfg.rootDir)
                rootDir = char(string(replayCfg.rootDir));
            else
                rootDir = ReplayProcessor.defaultRootDir();
            end
            if isfield(replayCfg, 'manifestFile') && ~isempty(replayCfg.manifestFile)
                manifestPath = char(string(replayCfg.manifestFile));
            else
                manifestPath = fullfile(rootDir, 'manifest.json');
            end
        end

        function manifest = loadManifest(manifestPath)
            if ~isfile(manifestPath)
                error('fth:Replay:ManifestMissing', ...
                    'Replay manifest ''%s'' does not exist.', manifestPath);
            end
            raw = fileread(manifestPath);
            manifest = jsondecode(raw);
        end

        function rawPath = resolveSourcePath(rootDir, sourceFile)
            % Resolve sources relative to processed/ first, then trajectories/.
            sourceFile = char(string(sourceFile));
            rawPath = fullfile(rootDir, sourceFile);
            if isfile(rawPath)
                return;
            end

            trajectoriesRoot = fileparts(rootDir);
            rawPath = fullfile(trajectoriesRoot, sourceFile);
        end

        function traj = processSingle(rawPath, replayId, entry)
            if ~isfile(rawPath)
                error('fth:Replay:RawFileMissing', ...
                    'Replay source file ''%s'' does not exist.', rawPath);
            end
            T = readtable(rawPath, 'VariableNamingRule', 'preserve');
            t = T.("elapsed_time");
            p = [T.("drone_x"), T.("drone_y"), T.("drone_z")];
            v_world = [T.("drone_velocity_linear_x"), T.("drone_velocity_linear_y"), T.("drone_velocity_linear_z")];
            omega_world = [T.("drone_velocity_angular_x"), T.("drone_velocity_angular_y"), T.("drone_velocity_angular_z")];
            rotCols = ["drone_rot[0]" "drone_rot[1]" "drone_rot[2]" ...
                       "drone_rot[3]" "drone_rot[4]" "drone_rot[5]" ...
                       "drone_rot[6]" "drone_rot[7]" "drone_rot[8]"];
            R = zeros(3, 3, height(T));
            v_b = zeros(size(v_world));
            omega_b = zeros(size(omega_world));
            for k = 1:height(T)
                % Dataset rotations are row-wise in the CSV; R maps body to world.
                R(:, :, k) = reshape(T{k, rotCols}, 3, 3).';
                v_b(k, :) = (R(:, :, k).' * v_world(k, :).').';
                omega_b(k, :) = (R(:, :, k).' * omega_world(k, :).').';
            end

            % The controller's acceleration is the time derivative of the body twist.
            a_b = ReplayProcessor.deriveSignal(v_b, t);
            alpha_b = ReplayProcessor.deriveSignal(omega_b, t);

            traj = struct();
            traj.t = t(:);
            traj.p = p;
            traj.v_b = v_b;
            traj.a_b = a_b;
            traj.omega_b = omega_b;
            traj.alpha_b = alpha_b;
            traj.R = R;
            traj.meta = struct( ...
                'id', replayId, ...
                'source_mode', char(string(entry.source_mode)), ...
                'source_file', char(string(entry.source_file)), ...
                'sampleRateHz', ReplayProcessor.estimateSampleRate(t), ...
                'tStart', t(1), ...
                'tEnd', t(end));
        end

        function rate = estimateSampleRate(t)
            dt = diff(t(:));
            if isempty(dt)
                rate = 0;
                return;
            end
            rate = 1 / median(dt);
        end

        function deriv = deriveSignal(values, t)
            deriv = zeros(size(values));
            if size(values, 1) < 2
                return;
            end
            for j = 1:size(values, 2)
                deriv(:, j) = gradient(values(:, j), t(:));
            end
        end
    end
end
