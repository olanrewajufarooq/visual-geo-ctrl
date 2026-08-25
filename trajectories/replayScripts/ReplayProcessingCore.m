classdef ReplayProcessingCore
    %REPLAYPROCESSOR Build and load canonical replay trajectory artifacts.

    methods (Static)
        function summary = processAll(rootDir, manifestPath, useParallel, methodOverride, clearCache)
            %PROCESSALL Convert manifest-listed raw CSV files into .mat artifacts.
            if nargin < 1 || isempty(rootDir)
                rootDir = ReplayProcessingCore.defaultRootDir();
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
            if nargin < 4 || isempty(methodOverride)
                methodOverride = '';
            else
                methodOverride = ReplayProcessingCore.normalizePostprocessingMethod( ...
                    methodOverride);
            end
            if nargin < 5 || isempty(clearCache)
                clearCache = true;
            else
                validateattributes(clearCache, {'logical', 'numeric'}, {'scalar'}, ...
                    mfilename, 'clearCache');
                clearCache = logical(clearCache);
            end

            manifest = ReplayProcessingCore.loadManifest(manifestPath);
            ids = fieldnames(manifest);
            n = numel(ids);
            rawPaths = cell(n, 1);
            outPaths = cell(n, 1);
            entries = cell(n, 1);
            for i = 1:n
                entries{i} = manifest.(ids{i});
                if ~isempty(methodOverride)
                    entries{i}.postprocessing.method = methodOverride;
                end
                rawPaths{i} = ReplayProcessingCore.resolveSourcePath(rootDir, entries{i}.source_file);
                outPaths{i} = fullfile(rootDir, char(string(entries{i}.artifact_file)));
                outDir = fileparts(outPaths{i});
                if ~isempty(outDir) && ~exist(outDir, 'dir')
                    mkdir(outDir);
                end
            end

            fprintf('[replay] Processing %d trajectories from %s\n', n, manifestPath);
            cached = false(n, 1);
            if ~clearCache
                for i = 1:n
                    cached(i) = ReplayProcessingCore.isCacheValid( ...
                        outPaths{i}, rawPaths{i}, entries{i});
                end
            end
            pending = find(~cached);
            elapsed = zeros(n, 1);
            errors = cell(n, 1);
            if useParallel && numel(pending) > 1
                fprintf('[replay] Parallel preprocessing enabled.\n');
                [pendingElapsed, pendingErrors] = ReplayProcessingCore.processAllParallel( ...
                    ids(pending), entries(pending), rawPaths(pending), outPaths(pending));
                elapsed(pending) = pendingElapsed;
                errors(pending) = pendingErrors;
            elseif ~isempty(pending)
                fprintf('[replay] Sequential preprocessing enabled.\n');
                [pendingElapsed, pendingErrors] = ReplayProcessingCore.processAllSequential( ...
                    ids(pending), entries(pending), rawPaths(pending), outPaths(pending));
                elapsed(pending) = pendingElapsed;
                errors(pending) = pendingErrors;
            else
                fprintf('[replay] Cache hit for all trajectories.\n');
            end

            failed = false(n, 1);
            for i = 1:n
                fprintf('[replay %d/%d] %s\n', i, n, ids{i});
                fprintf('  source: %s\n', rawPaths{i});
                if cached(i)
                    fprintf('  cache: reused %s\n', outPaths{i});
                elseif isempty(errors{i})
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

            summary = struct('processedCount', sum(~cached), ...
                'cachedCount', sum(cached), 'totalCount', n, ...
                'manifestPath', manifestPath, ...
                'parallel', useParallel && numel(pending) > 1, ...
                'clearCache', clearCache);
            fprintf('[replay] Completed %d trajectories (%d processed, %d cached).\n', ...
                summary.totalCount, summary.processedCount, summary.cachedCount);
        end

        function entry = loadEntry(replayCfg)
            %LOADENTRY Resolve one replay manifest entry from config.
            [rootDir, manifestPath, replayId] = ReplayProcessingCore.resolveReplayConfig(replayCfg);
            manifest = ReplayProcessingCore.loadManifest(manifestPath);
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
            entry = ReplayProcessingCore.loadEntry(replayCfg);
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
                traj = ReplayProcessingCore.loadArtifact(cfg.traj.replay);
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
        function method = normalizePostprocessingMethod(method)
            method = lower(char(string(method)));
            switch method
                case 'poly'
                    method = 'legacy-local-polynomial-world-derivative';
                case 'wnoj'
                    method = 'wnoj-se3-batch-v1';
                case {'legacy-local-polynomial-world-derivative', 'wnoj-se3-batch-v1'}
                    % Permit the internal names for programmatic callers.
                otherwise
                    error('fth:Replay:UnknownProcessingMethod', ...
                        ['Unknown replay processing method ''%s''. ' ...
                         'Expected ''wnoj'' or ''poly''.'], method);
            end
        end

        function [elapsed, errors] = processAllSequential(ids, entries, rawPaths, outPaths)
            n = numel(ids);
            elapsed = zeros(n, 1);
            errors = cell(n, 1);
            for i = 1:n
                fileTimer = tic;
                try
                    traj = ReplayProcessingCore.processSingle(rawPaths{i}, ids{i}, entries{i});
                    traj.meta.postprocessingCacheKey = ...
                        ReplayProcessingCore.postprocessingCacheKey(entries{i});
                    writeReplayArtifact(outPaths{i}, traj);
                catch exception
                    errors{i} = ReplayProcessingCore.formatException(exception);
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
                    traj = ReplayProcessingCore.processSingle(rawPaths{i}, ids{i}, entries{i});
                    traj.meta.postprocessingCacheKey = ...
                        ReplayProcessingCore.postprocessingCacheKey(entries{i});
                    writeReplayArtifact(outPaths{i}, traj);
                catch exception
                    errors{i} = ReplayProcessingCore.formatException(exception);
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
                rootDir = ReplayProcessingCore.defaultRootDir();
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
            traj = ReplayKinematics.processSingle(rawPath, replayId, entry);
        end

        function valid = isCacheValid(artifactPath, rawPath, entry)
            valid = false;
            if ~isfile(artifactPath)
                return;
            end
            if isfile(rawPath)
                rawInfo = dir(rawPath);
                artifactInfo = dir(artifactPath);
                if rawInfo.datenum > artifactInfo.datenum
                    return;
                end
            end
            try
                data = load(artifactPath, 'traj');
                valid = isfield(data, 'traj') && isfield(data.traj, 'meta') && ...
                    isfield(data.traj.meta, 'postprocessingCacheKey') && ...
                    strcmp(data.traj.meta.postprocessingCacheKey, ...
                    ReplayProcessingCore.postprocessingCacheKey(entry));
            catch
                valid = false;
            end
        end

        function key = postprocessingCacheKey(entry)
            method = 'legacy-local-polynomial-world-derivative';
            if isfield(entry, 'postprocessing') && ...
                    isstruct(entry.postprocessing) && ...
                    isfield(entry.postprocessing, 'method') && ...
                    ~isempty(entry.postprocessing.method)
                method = ReplayProcessingCore.normalizePostprocessingMethod( ...
                    entry.postprocessing.method);
            end
            payload = struct('method', method);
            if isfield(entry, 'postprocessing') && ...
                    isstruct(entry.postprocessing) && ...
                    isfield(entry.postprocessing, 'wnoj')
                payload.wnoj = entry.postprocessing.wnoj;
            end
            key = jsonencode(payload);
        end
    end
end
