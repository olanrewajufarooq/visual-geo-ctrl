classdef ResultsManager
    %RESULTSMANAGER Handles results directory setup, persistence, and loading.
    %   Manages creation of timestamped results directories, saving/loading
    %   simulation data (.mat), and text file I/O for run artifacts.

    methods (Static)
        function resultsDir = createResultsDir(cfg, repoRoot, runName)
            %CREATERESULTSDIR Create a run-specific results folder.
            %   Inputs:
            %     cfg - config with controller and traj fields.
            %     repoRoot - repository root path.
            %     runName - pre-built run name string.
            %   Output:
            %     resultsDir - full path to the new results directory.
            baseDir = fullfile(repoRoot, 'results');
            if ~exist(baseDir, 'dir')
                mkdir(baseDir);
            end

            subfolder = 'nominal';
            if isfield(cfg.controller, 'adaptation') && ~strcmpi(cfg.controller.adaptation, 'none')
                subfolder = 'adaptive';
            end

            runDir = fullfile(baseDir, subfolder);
            if ~exist(runDir, 'dir')
                mkdir(runDir);
            end

            resultsDir = fullfile(runDir, runName);
            mkdir(resultsDir);
        end

        function runName = buildRunName(cfg, isBatch)
            %BUILDRUNNAME Build a timestamped run name from config.
            %   Inputs:
            %     cfg - config with traj and controller fields.
            %     isBatch - true for batch mode.
            %   Output:
            %     runName - string like '20260426_123456_inf3d_ff_lie'.
            timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
            trajName = vt.sim.NamingUtils.batchTrajectoryLabel(cfg);
            if isBatch
                runName = sprintf('%s_%s', timestamp, trajName);
            else
                ctrlType = vt.sim.NamingUtils.controllerLabel(cfg);
                potential = vt.sim.NamingUtils.potentialLabel(cfg);
                runName = sprintf('%s_%s_%s_%s', timestamp, trajName, ctrlType, potential);
            end
        end

        function persistRun(resultsDir, logs, metrics, est, runInfo, cfgSnapshot)
            %PERSISTRUN Save finalized run data to sim_data.mat.
            %   Inputs: resultsDir, logs, metrics, est, runInfo, cfgSnapshot.
            dataPath = fullfile(resultsDir, 'sim_data.mat');
            save(dataPath, 'logs', 'metrics', 'est', 'runInfo', 'cfgSnapshot');
            fprintf('Run data saved to: %s\n', dataPath);
        end

        function saved = loadRun(path)
            %LOADRUN Load saved run data from disk.
            %   Input: path - resultsDir or direct sim_data.mat path.
            %   Output: saved struct with logs, metrics, est, runInfo, cfgSnapshot.
            [resultsDir, dataPath] = vt.sim.ResultsManager.resolveRunDataPath(path);
            if ~exist(dataPath, 'file')
                error('ResultsManager:MissingSavedData', 'Missing sim_data.mat in %s', resultsDir);
            end
            saved = load(dataPath, 'logs', 'metrics', 'est', 'runInfo', 'cfgSnapshot');
        end

        function writeMetricsFile(resultsDir, metrics, runInfo, cfgSnapshot)
            %WRITEMETRICSFILE Save lightweight run metrics for reporting.
            runLabel = 'Run';
            if isprop(cfgSnapshot, 'sim') && isfield(cfgSnapshot.sim, 'batchRunIndex') ...
                    && ~isempty(cfgSnapshot.sim.batchRunIndex)
                runLabel = sprintf('Run %d', cfgSnapshot.sim.batchRunIndex);
            end

            entry = struct();
            entry.trajectory = char(string(cfgSnapshot.traj.name));
            entry.run_label = runLabel;
            entry.is_adaptive = logical(runInfo.isAdaptive);
            entry.track_rmse = metrics.combined.rmse_total;
            entry.track_score = metrics.combined.tracking_score;
            entry.mass_rmse = NaN;
            entry.mass_nrmse = NaN;
            entry.mass_tracking_score = NaN;
            entry.mass_ident_metric = NaN;
            entry.mass_ident_score = NaN;
            entry.cog_rmse = NaN;
            entry.cog_nrmse = NaN;
            entry.cog_tracking_score = NaN;
            entry.cog_ident_metric = NaN;
            entry.cog_ident_score = NaN;
            entry.inertia_rmse = NaN;
            entry.inertia_nrmse = NaN;
            entry.inertia_tracking_score = NaN;
            entry.inertia_ident_metric = NaN;
            entry.inertia_ident_score = NaN;
            if isfield(metrics, 'parameters')
                entry.mass_rmse = metrics.parameters.mass.rmse;
                entry.mass_nrmse = metrics.parameters.mass.nrmse;
                entry.mass_tracking_score = metrics.parameters.mass.tracking_score;
                entry.cog_rmse = metrics.parameters.cog.rmse_total;
                entry.cog_nrmse = metrics.parameters.cog.nrmse_total;
                entry.cog_tracking_score = metrics.parameters.cog.tracking_score;
                entry.inertia_rmse = metrics.parameters.inertia.rmse_total;
                entry.inertia_nrmse = metrics.parameters.inertia.nrmse_total;
                entry.inertia_tracking_score = metrics.parameters.inertia.tracking_score;
                if isfield(metrics.parameters, 'identifiability')
                    entry.mass_ident_metric = metrics.parameters.identifiability.mass.sigma_min;
                    entry.mass_ident_score = metrics.parameters.identifiability.mass.score;
                    entry.cog_ident_metric = metrics.parameters.identifiability.mcog.sigma_min;
                    entry.cog_ident_score = metrics.parameters.identifiability.mcog.score;
                    entry.inertia_ident_metric = metrics.parameters.identifiability.inertia.sigma_min;
                    entry.inertia_ident_score = metrics.parameters.identifiability.inertia.score;
                end
            end

            lines = { ...
                sprintf('trajectory=%s', entry.trajectory), ...
                sprintf('run_label=%s', entry.run_label), ...
                sprintf('is_adaptive=%s', vt.sim.ResultsManager.boolText(entry.is_adaptive)), ...
                sprintf('track_rmse=%s', vt.sim.ResultsManager.metricText(entry.track_rmse, 4)), ...
                sprintf('track_score=%s', vt.sim.ResultsManager.metricText(entry.track_score, 2)), ...
                sprintf('mass_rmse=%s', vt.sim.ResultsManager.metricText(entry.mass_rmse, 4)), ...
                sprintf('mass_nrmse=%s', vt.sim.ResultsManager.metricText(entry.mass_nrmse, 4)), ...
                sprintf('mass_tracking_score=%s', vt.sim.ResultsManager.metricText(entry.mass_tracking_score, 2)), ...
                sprintf('mass_ident_metric=%s', vt.sim.ResultsManager.metricText(entry.mass_ident_metric, 4)), ...
                sprintf('mass_ident_score=%s', vt.sim.ResultsManager.metricText(entry.mass_ident_score, 2)), ...
                sprintf('cog_rmse=%s', vt.sim.ResultsManager.metricText(entry.cog_rmse, 4)), ...
                sprintf('cog_nrmse=%s', vt.sim.ResultsManager.metricText(entry.cog_nrmse, 4)), ...
                sprintf('cog_tracking_score=%s', vt.sim.ResultsManager.metricText(entry.cog_tracking_score, 2)), ...
                sprintf('cog_ident_metric=%s', vt.sim.ResultsManager.metricText(entry.cog_ident_metric, 4)), ...
                sprintf('cog_ident_score=%s', vt.sim.ResultsManager.metricText(entry.cog_ident_score, 2)), ...
                sprintf('inertia_rmse=%s', vt.sim.ResultsManager.metricText(entry.inertia_rmse, 4)), ...
                sprintf('inertia_nrmse=%s', vt.sim.ResultsManager.metricText(entry.inertia_nrmse, 4)), ...
                sprintf('inertia_tracking_score=%s', vt.sim.ResultsManager.metricText(entry.inertia_tracking_score, 2)), ...
                sprintf('inertia_ident_metric=%s', vt.sim.ResultsManager.metricText(entry.inertia_ident_metric, 4)), ...
                sprintf('inertia_ident_score=%s', vt.sim.ResultsManager.metricText(entry.inertia_ident_score, 2))};
            vt.sim.ResultsManager.writeTextFile(fullfile(resultsDir, 'metrics.txt'), strjoin(lines, newline));
        end

        function metricsEntry = loadMetricsFile(path)
            %LOADMETRICSFILE Load lightweight metrics from metrics.txt.
            [~, metricsPath] = vt.sim.ResultsManager.resolveMetricsPath(path);
            if ~exist(metricsPath, 'file')
                error('ResultsManager:MissingMetricsFile', 'Missing metrics.txt in %s', path);
            end
            content = vt.sim.ResultsManager.readTextFile(metricsPath);
            lines = regexp(content, '\r?\n', 'split');
            metricsEntry = struct();
            for i = 1:numel(lines)
                line = strtrim(lines{i});
                if isempty(line)
                    continue;
                end
                tokens = regexp(line, '^([^=]+)=(.*)$', 'tokens', 'once');
                if isempty(tokens)
                    continue;
                end
                key = strtrim(tokens{1});
                value = strtrim(tokens{2});
                switch key
                    case {'trajectory', 'run_label'}
                        metricsEntry.(key) = value;
                    case 'is_adaptive'
                        metricsEntry.(key) = strcmpi(value, 'true');
                    otherwise
                        metricsEntry.(key) = vt.sim.ResultsManager.metricValue(value);
                end
            end
        end

        function writeTextFile(path, content)
            %WRITETEXTFILE Write UTF-8 text content to a file.
            fid = fopen(path, 'w');
            if fid < 0
                error('ResultsManager:WriteFailed', 'Unable to write file: %s', path);
            end
            cleanup = onCleanup(@() fclose(fid));
            fprintf(fid, '%s\n', content);
        end

        function content = readTextFile(path)
            %READTEXTFILE Read a UTF-8 text file.
            if ~exist(path, 'file')
                content = '';
                return;
            end
            content = fileread(path);
        end

        function plotSavedRun(path, plotType, displayPlots)
            %PLOTSAVEDRUN Generate plots for one saved run directory or sim_data.mat.
            if nargin < 3 || isempty(displayPlots)
                displayPlots = false;
            end
            if string(plotType) == "none"
                return;
            end
            [resultsDir, ~] = vt.sim.ResultsManager.resolveRunDataPath(path);
            saved = vt.sim.ResultsManager.loadRun(path);
            vt.sim.ResultsManager.plotRunData(resultsDir, ...
                saved.logs, saved.est, saved.runInfo, saved.cfgSnapshot, plotType, displayPlots);
        end

        function plotRunData(resultsDir, logs, est, runInfo, cfgSnapshot, plotType, displayPlots)
            %PLOTRUNDATA Generate plots for finalized in-memory run data.
            isAdaptive = logical(runInfo.isAdaptive);
            layoutType = 'row-major';
            if isprop(cfgSnapshot, 'viz') && isfield(cfgSnapshot.viz, 'plotLayout') && ~isempty(cfgSnapshot.viz.plotLayout)
                layoutType = cfgSnapshot.viz.plotLayout;
            end

            plotterObj = vt.plot.Plotter(resultsDir, struct('savePng', true, 'duration', runInfo.duration));
            previousVisible = get(groot, 'defaultFigureVisible');
            existingFigures = findall(groot, 'Type', 'figure');
            cleanup = onCleanup(@() set(groot, 'defaultFigureVisible', previousVisible));
            cleanupObj = cleanup; %#ok<NASGU>
            if displayPlots
                set(groot, 'defaultFigureVisible', 'on');
            else
                set(groot, 'defaultFigureVisible', 'off');
            end

            if isAdaptive
                summaryFig = figure('Name','Final Summary - Adaptive','Position',[50 50 1400 900]);
                plotterObj.plotSummaryAdaptive(logs, est, summaryFig, layoutType);
            else
                summaryFig = figure('Name','Final Summary - Nominal','Position',[100 100 1400 600]);
                plotterObj.plotSummaryNominal(logs, summaryFig, layoutType);
            end

            if string(plotType) == "all"
                if isAdaptive
                    plotterObj.plotStandaloneSubplotsAdaptive(logs, est);
                    plotterObj.plotStackedEstimation(est);
                    plotterObj.plotStackedInertia(est);
                else
                    plotterObj.plotStandaloneSubplotsNominal(logs);
                end
                plotterObj.plotStackedAllState(logs);
                plotterObj.plotStackedPositionOrientation(logs);
                plotterObj.plotStackedVelocity(logs);
                plotterObj.plotStackedWrench(logs);
            end

            if ~displayPlots
                createdFigures = setdiff(findall(groot, 'Type', 'figure'), existingFigures);
                if ~isempty(createdFigures)
                    close(createdFigures(ishandle(createdFigures)));
                end
            end
        end

        function dirs = findChildResultDirs(resultsDir)
            %FINDCHILDRESULTDIRS Discover saved batch child result directories.
            dirs = {};
            if ~exist(resultsDir, 'dir')
                return;
            end
            listing = dir(fullfile(resultsDir, '**', 'metrics.txt'));
            if isempty(listing)
                listing = dir(fullfile(resultsDir, '**', 'sim_data.mat'));
            end
            if isempty(listing)
                return;
            end
            dirs = cell(numel(listing), 1);
            for i = 1:numel(listing)
                dirs{i} = listing(i).folder;
            end
            dirs = sort(unique(dirs));
        end

        function root = repoRoot()
            %REPOROOT Return repository root path.
            p = mfilename('fullpath');
            root = fileparts(fileparts(fileparts(fileparts(p))));
        end
    end

    methods (Static, Access = private)
        function [resultsDir, dataPath] = resolveRunDataPath(path)
            %RESOLVERUNDATAPATH Resolve a results dir and sim_data.mat path.
            if isfolder(path)
                resultsDir = path;
                dataPath = fullfile(resultsDir, 'sim_data.mat');
                return;
            end
            if exist(path, 'file') == 2
                [resultsDir, fileName, ext] = fileparts(path);
                if strcmpi([fileName ext], 'sim_data.mat')
                    dataPath = path;
                    return;
                end
            end
            error('ResultsManager:InvalidRunDataPath', ...
                'Path must be a results directory or a direct sim_data.mat file path.');
        end

        function [resultsDir, metricsPath] = resolveMetricsPath(path)
            %RESOLVEMETRICSPATH Resolve a results dir and metrics.txt path.
            if isfolder(path)
                resultsDir = path;
                metricsPath = fullfile(resultsDir, 'metrics.txt');
                return;
            end
            if exist(path, 'file') == 2
                [resultsDir, fileName, ext] = fileparts(path);
                if strcmpi([fileName ext], 'metrics.txt')
                    metricsPath = path;
                    return;
                end
            end
            error('ResultsManager:InvalidMetricsPath', ...
                'Path must be a results directory or a direct metrics.txt file path.');
        end

        function text = metricText(value, decimals)
            %METRICTEXT Format a metric value for metrics.txt.
            if ~isfinite(value)
                text = 'N/A';
                return;
            end
            text = sprintf(['%0.' num2str(decimals) 'f'], value);
        end

        function value = metricValue(text)
            %METRICVALUE Parse a metric value from metrics.txt.
            if strcmpi(text, 'N/A')
                value = NaN;
                return;
            end
            value = str2double(text);
        end

        function text = boolText(flag)
            %BOOLTEXT Convert logical flag to lowercase text.
            if flag
                text = 'true';
            else
                text = 'false';
            end
        end
    end
end
