classdef ReplayProcessor
    %REPLAYPROCESSOR Build and load canonical replay trajectory artifacts.

    methods (Static)
        function summary = processAll(rootDir, manifestPath, useParallel)
            %PROCESSALL Convert manifest-listed raw CSV files into .mat artifacts.
            if nargin < 1 || isempty(rootDir)
                rootDir = ReplayProcessor.defaultRootDir();
            end
            if nargin < 2 || isempty(manifestPath)
                manifestPath = fullfile(rootDir, 'manifest.json');
            end
            if nargin < 3 || isempty(useParallel)
                useParallel = ~isempty(ver('parallel'));
            else
                useParallel = logical(useParallel);
                if useParallel && isempty(ver('parallel'))
                    warning('fth:Replay:ParallelUnavailable', ...
                        'Parallel Computing Toolbox is unavailable; using sequential preprocessing.');
                    useParallel = false;
                end
            end

            manifest = ReplayProcessor.loadManifest(manifestPath);
            ids = fieldnames(manifest);
            n = numel(ids);
            rawPaths = cell(n, 1);
            outPaths = cell(n, 1);
            entries = cell(n, 1);
            for i = 1:n
                entries{i} = manifest.(ids{i});
                rawPaths{i} = ReplayProcessor.resolveSourcePath(rootDir, entries{i}.source_file);
                outPaths{i} = fullfile(rootDir, char(string(entries{i}.artifact_file)));
                outDir = fileparts(outPaths{i});
                if ~isempty(outDir) && ~exist(outDir, 'dir')
                    mkdir(outDir);
                end
            end

            fprintf('[replay] Processing %d trajectories from %s\n', n, manifestPath);
            if useParallel && n > 1
                fprintf('[replay] Parallel preprocessing enabled.\n');
                [elapsed, errors] = ReplayProcessor.processAllParallel( ...
                    ids, entries, rawPaths, outPaths);
            else
                fprintf('[replay] Sequential preprocessing enabled.\n');
                [elapsed, errors] = ReplayProcessor.processAllSequential( ...
                    ids, entries, rawPaths, outPaths);
            end

            failed = false(n, 1);
            for i = 1:n
                fprintf('[replay %d/%d] %s\n', i, n, ids{i});
                fprintf('  source: %s\n', rawPaths{i});
                if isempty(errors{i})
                    fprintf('  artifact: %s (%.1f s)\n', outPaths{i}, elapsed(i));
                else
                    failed(i) = true;
                    fprintf('  FAILED after %.1f s\n', elapsed(i));
                    fprintf('  error: %s\n', errors{i});
                end
            end

            if any(failed)
                failedIds = ids(failed);
                error('fth:Replay:ProcessingFailed', ...
                    'Replay preprocessing failed for: %s', strjoin(failedIds, ', '));
            end

            summary = struct('processedCount', n, ...
                'manifestPath', manifestPath, 'parallel', useParallel && n > 1);
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
        function [elapsed, errors] = processAllSequential(ids, entries, rawPaths, outPaths)
            n = numel(ids);
            elapsed = zeros(n, 1);
            errors = cell(n, 1);
            for i = 1:n
                fileTimer = tic;
                try
                    traj = ReplayProcessor.processSingle(rawPaths{i}, ids{i}, entries{i});
                    writeReplayArtifact(outPaths{i}, traj);
                catch exception
                    errors{i} = ReplayProcessor.formatException(exception);
                end
                elapsed(i) = toc(fileTimer);
            end
        end

        function [elapsed, errors] = processAllParallel(ids, entries, rawPaths, outPaths)
            n = numel(ids);
            elapsed = zeros(n, 1);
            errors = cell(n, 1);
            parfor i = 1:n
                fileTimer = tic;
                try
                    traj = ReplayProcessor.processSingle(rawPaths{i}, ids{i}, entries{i});
                    writeReplayArtifact(outPaths{i}, traj);
                catch exception
                    errors{i} = ReplayProcessor.formatException(exception);
                end
                elapsed(i) = toc(fileTimer);
            end
        end

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

        function text = formatException(exception)
            if isempty(exception.stack)
                text = exception.message;
                return;
            end
            frame = exception.stack(1);
            text = sprintf('%s (at %s:%d)', exception.message, frame.name, frame.line);
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
                % The CSV stores the body-to-world rotation in column-major form.
                R_body_to_world = reshape(T{k, rotCols}, 3, 3).';
                R_world_to_body = R_body_to_world.';
                R(:, :, k) = R_body_to_world;

                % SimRunner expects the translational and angular twist in body
                % coordinates, so convert each world-frame vector explicitly.
                v_world_k = v_world(k, :).';
                omega_world_k = omega_world(k, :).';
                v_b(k, :) = (R_world_to_body * v_world_k).';
                omega_b(k, :) = (R_world_to_body * omega_world_k).';
            end

            % Estimate derivatives only after the complete velocity signals have
            % been assembled. The local polynomial fit suppresses sample-level
            % MoCap/interpolation noise without introducing a causal delay.
            derivativeWindowSamples = 201;  % 402 ms at the dataset's 500 Hz rate
            v_world_dot = ReplayProcessor.differentiateSignal( ...
                v_world, t, derivativeWindowSamples);
            omega_world_dot = ReplayProcessor.differentiateSignal( ...
                omega_world, t, derivativeWindowSamples);

            % The simulator uses the time derivative of the body twist. For
            % R = body-to-world and v_b = R' * v_world:
            %   d(v_b)/dt = R' * d(v_world)/dt - omega_b x v_b
            % The angular transport term vanishes because omega_b x omega_b=0.
            a_b = zeros(size(v_b));
            alpha_b = zeros(size(omega_b));
            for k = 1:height(T)
                R_world_to_body = R(:, :, k).';
                v_body_k = v_b(k, :).';
                omega_body_k = omega_b(k, :).';
                a_world_k = v_world_dot(k, :).';
                omega_world_dot_k = omega_world_dot(k, :).';

                a_b(k, :) = (R_world_to_body * a_world_k - ...
                    cross(omega_body_k, v_body_k)).';
                alpha_b(k, :) = (R_world_to_body * omega_world_dot_k).';
            end

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
                'accelerationMethod', 'local-polynomial-world-derivative', ...
                'accelerationWindowSamples', derivativeWindowSamples, ...
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

        function deriv = differentiateSignal(values, t, windowSamples)
            %DIFFERENTIATESIGNAL Estimate d(values)/dt with local cubic fits.
            %   The fit is centered at every sample, so this is a smoothed
            %   offline derivative rather than a delayed causal filter.
            n = size(values, 1);
            deriv = zeros(size(values));
            if n < 2
                return;
            end

            t = t(:);
            if any(~isfinite(t)) || any(diff(t) <= 0)
                error('fth:Replay:InvalidTime', ...
                    'Replay elapsed_time must be finite and strictly increasing.');
            end

            windowSamples = min(n, max(5, round(windowSamples)));
            if mod(windowSamples, 2) == 0
                windowSamples = windowSamples - 1;
            end
            halfWindow = floor(windowSamples / 2);

            for i = 1:n
                first = max(1, i - halfWindow);
                last = min(n, i + halfWindow);
                indices = first:last;
                offsets = t(indices) - t(i);
                timeScale = max(abs(offsets));
                if timeScale == 0
                    continue;
                end

                normalizedOffsets = offsets / timeScale;
                localDegree = min(3, numel(indices) - 1);
                design = zeros(numel(indices), localDegree + 1);
                for degree = 0:localDegree
                    design(:, degree + 1) = normalizedOffsets .^ degree;
                end

                basisDerivative = zeros(localDegree + 1, 1);
                basisDerivative(2) = 1;
                % d/dt at the center = e_1' * pinv(design) * samples.
                weights = (pinv(design).' * basisDerivative) / timeScale;
                deriv(i, :) = weights.' * values(indices, :);
            end
        end
    end
end
