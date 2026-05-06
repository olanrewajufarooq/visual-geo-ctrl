classdef BatchRunner < handle
    %BATCHRUNNER Orchestrates multi-trajectory and multi-gain batch runs.
    %   Manages child SimRunner creation, per-run console capture, aggregate
    %   report generation, and batch plot dispatch.
    %
    %   Usage:
    %     br = vt.sim.BatchRunner(cfg, resultsDir, batchSize);
    %     br.runAll(runArgs);
    %     br.plotAll('summary');

    properties (Access = private)
        cfg
        resultsDir
        batchSize
        childDirs
        console
    end

    methods
        function obj = BatchRunner(cfg, resultsDir, batchSize)
            %BATCHRUNNER Create a batch runner.
            %   Inputs:
            %     cfg - vt.config.Config instance.
            %     resultsDir - root results directory for this batch.
            %     batchSize - total number of runs.
            obj.cfg = cfg;
            obj.resultsDir = resultsDir;
            obj.batchSize = batchSize;
            obj.childDirs = {};
            obj.console = vt.sim.ConsoleCapture();
        end

        function runAll(obj, runArgs)
            %RUNALL Execute all batch runs with per-run console capture.
            %   Input: runArgs - cell array of arguments for child.run().
            cfgs = obj.cfg.expandBatchConfigs(obj.resultsDir);
            obj.childDirs = cell(obj.batchSize, 1);
            for i = 1:obj.batchSize
                child = vt.sim.SimRunner(cfgs{i});
                runLog = obj.console.capture(@() obj.executeChild(child, runArgs));
                childLogPath = fullfile(child.resultsDir, 'command_window.txt');
                vt.sim.ResultsManager.writeTextFile(childLogPath, strtrim(runLog));
                obj.childDirs{i} = child.resultsDir;
                clear child
            end
            obj.writeAggregateArtifacts();
            fprintf('Batch results saved to: %s\n', obj.resultsDir);
        end

        function plotAll(obj, plotType, displayPlots)
            %PLOTALL Generate plots for each saved batch run.
            if nargin < 3 || isempty(displayPlots)
                displayPlots = false;
            end
            if isempty(obj.childDirs)
                obj.childDirs = vt.sim.ResultsManager.findChildResultDirs(obj.resultsDir);
            end
            if isempty(obj.childDirs)
                error('BatchRunner:NotRun', 'Batch simulation has not been run yet.');
            end
            for i = 1:numel(obj.childDirs)
                vt.sim.ResultsManager.plotSavedRun(obj.childDirs{i}, char(plotType), displayPlots);
            end
            obj.writeAggregateArtifacts();
        end

        function dirs = getChildDirs(obj)
            %GETCHILDDIRS Return the list of child result directories.
            dirs = obj.childDirs;
        end
    end

    methods (Access = private)
        function executeChild(~, child, runArgs)
            %EXECUTECHILD Run setup and simulation for one child runner.
            child.setup();
            child.run(runArgs{:});
        end

        function writeAggregateArtifacts(obj)
            %WRITEAGGREGATEARTIFACTS Rebuild aggregate logs and reports from saved runs.
            if isempty(obj.childDirs)
                obj.childDirs = vt.sim.ResultsManager.findChildResultDirs(obj.resultsDir);
            end
            aggregateChunks = cell(numel(obj.childDirs), 1);
            for i = 1:numel(obj.childDirs)
                metricsEntry = vt.sim.ResultsManager.loadMetricsFile(obj.childDirs{i});
                childLogPath = fullfile(obj.childDirs{i}, 'command_window.txt');
                childLog = vt.sim.ResultsManager.readTextFile(childLogPath);
                aggregateChunks{i} = sprintf('%s%s\n', ...
                    vt.sim.ConsoleFormatter.runBanner( ...
                    metricsEntry.trajectory, metricsEntry.run_label, metricsEntry.is_adaptive), ...
                    strtrim(childLog));
            end
            aggregatePath = fullfile(obj.resultsDir, 'command_window.txt');
            vt.sim.ResultsManager.writeTextFile(aggregatePath, strjoin(aggregateChunks, newline));
            if obj.isAdaptiveBatch()
                summaryPath = fullfile(obj.resultsDir, 'adaptive_report.txt');
                vt.sim.ResultsManager.writeTextFile(summaryPath, obj.buildSummaryTable());
                identPath = fullfile(obj.resultsDir, 'ident_report.txt');
                vt.sim.ResultsManager.writeTextFile(identPath, obj.buildIdentifiabilityReport());
            end
        end

        function tf = isAdaptiveBatch(obj)
            %ISADAPTIVEBATCH Return true when all saved batch runs use adaptation.
            tf = ~isempty(obj.childDirs);
            if ~tf, return; end
            for i = 1:numel(obj.childDirs)
                metricsEntry = vt.sim.ResultsManager.loadMetricsFile(obj.childDirs{i});
                if ~isfield(metricsEntry, 'is_adaptive') || ~metricsEntry.is_adaptive
                    tf = false;
                    return;
                end
            end
        end

        function tableText = buildSummaryTable(obj)
            %BUILDSUMMARYTABLE Build an aligned summary table from saved runs.
            nRuns = numel(obj.childDirs);
            headers = {'Trajectory', 'Run', 'Track RMSE', 'Track Score', ...
                'Mass RMSE', 'Mass NRMSE', 'Mass Tracking Score', 'Mass Ident Metric', 'Mass Ident Score', ...
                'CoG RMSE', 'CoG NRMSE', 'CoG Tracking Score', 'CoG Ident Metric', 'CoG Ident Score', ...
                'Inertia RMSE', 'Inertia NRMSE', 'Inertia Tracking Score', 'Inertia Ident Metric', 'Inertia Ident Score'};
            rawRows = cell(nRuns, numel(headers));
            numericValues = nan(nRuns, numel(headers));
            trajectoryNames = cell(nRuns, 1);
            betterIsLower = [false, false, true, false, ...
                true, true, false, false, false, ...
                true, true, false, false, false, ...
                true, true, false, false, false];

            for i = 1:nRuns
                metrics = vt.sim.ResultsManager.loadMetricsFile(obj.childDirs{i});
                trajectoryNames{i} = metrics.trajectory;
                rawRows{i,1} = trajectoryNames{i};
                rawRows{i,2} = metrics.run_label;

                rawRows{i,3} = obj.fmtMetric(metrics.track_rmse, 4);
                rawRows{i,4} = obj.fmtMetric(metrics.track_score, 2);
                numericValues(i,3) = metrics.track_rmse;
                numericValues(i,4) = metrics.track_score;

                if metrics.is_adaptive
                    rawRows{i,5} = obj.fmtMetric(metrics.mass_rmse, 4);
                    rawRows{i,6} = obj.fmtMetric(metrics.mass_nrmse, 4);
                    rawRows{i,7} = obj.fmtMetric(metrics.mass_tracking_score, 2);
                    rawRows{i,8} = obj.fmtMetric(metrics.mass_ident_metric, 4);
                    rawRows{i,9} = obj.fmtMetric(metrics.mass_ident_score, 2);
                    rawRows{i,10} = obj.fmtMetric(metrics.cog_rmse, 4);
                    rawRows{i,11} = obj.fmtMetric(metrics.cog_nrmse, 4);
                    rawRows{i,12} = obj.fmtMetric(metrics.cog_tracking_score, 2);
                    rawRows{i,13} = obj.fmtMetric(metrics.cog_ident_metric, 4);
                    rawRows{i,14} = obj.fmtMetric(metrics.cog_ident_score, 2);
                    rawRows{i,15} = obj.fmtMetric(metrics.inertia_rmse, 4);
                    rawRows{i,16} = obj.fmtMetric(metrics.inertia_nrmse, 4);
                    rawRows{i,17} = obj.fmtMetric(metrics.inertia_tracking_score, 2);
                    rawRows{i,18} = obj.fmtMetric(metrics.inertia_ident_metric, 4);
                    rawRows{i,19} = obj.fmtMetric(metrics.inertia_ident_score, 2);
                    numericValues(i,5) = metrics.mass_rmse;
                    numericValues(i,6) = metrics.mass_nrmse;
                    numericValues(i,7) = metrics.mass_tracking_score;
                    numericValues(i,8) = metrics.mass_ident_metric;
                    numericValues(i,9) = metrics.mass_ident_score;
                    numericValues(i,10) = metrics.cog_rmse;
                    numericValues(i,11) = metrics.cog_nrmse;
                    numericValues(i,12) = metrics.cog_tracking_score;
                    numericValues(i,13) = metrics.cog_ident_metric;
                    numericValues(i,14) = metrics.cog_ident_score;
                    numericValues(i,15) = metrics.inertia_rmse;
                    numericValues(i,16) = metrics.inertia_nrmse;
                    numericValues(i,17) = metrics.inertia_tracking_score;
                    numericValues(i,18) = metrics.inertia_ident_metric;
                    numericValues(i,19) = metrics.inertia_ident_score;
                else
                    rawRows(i,5:19) = {'N/A'};
                end
            end

            [~, ~, trajectoryGroups] = unique(trajectoryNames, 'stable');
            for groupId = 1:max(trajectoryGroups)
                groupRows = find(trajectoryGroups == groupId);
                for col = 3:numel(headers)
                    values = numericValues(groupRows, col);
                    validMask = isfinite(values);
                    if ~any(validMask), continue; end
                    validValues = values(validMask);
                    if betterIsLower(col)
                        bestValue = min(validValues);
                    else
                        bestValue = max(validValues);
                    end
                    bestMask = validMask & abs(values - bestValue) <= max(1e-12, abs(bestValue) * 1e-12);
                    bestRows = groupRows(bestMask);
                    for row = bestRows.'
                        rawRows{row,col} = sprintf('%s (best)', rawRows{row,col});
                    end
                end
            end

            widths = cellfun(@strlength, headers);
            for col = 1:numel(headers)
                for row = 1:nRuns
                    widths(col) = max(widths(col), strlength(string(rawRows{row,col})));
                end
            end

            lines = strings(nRuns + 5, 1);
            lineIdx = 1;
            lines(lineIdx) = "Batch Run Summary"; lineIdx = lineIdx + 1;
            lines(lineIdx) = obj.buildSep(widths); lineIdx = lineIdx + 1;
            lines(lineIdx) = obj.buildRow(headers, widths); lineIdx = lineIdx + 1;
            lines(lineIdx) = obj.buildSep(widths); lineIdx = lineIdx + 1;
            for row = 1:nRuns
                lines(lineIdx) = obj.buildRow(rawRows(row,:), widths);
                lineIdx = lineIdx + 1;
            end
            lines(lineIdx) = obj.buildSep(widths);
            tableText = strjoin(cellstr(lines), newline);
        end

        function reportText = buildIdentifiabilityReport(obj)
            %BUILDIDENTIFIABILITYREPORT Build a gain-first identifiability report.
            entries = obj.collectIdentifiabilityEntries();
            nonzeroMask = ~[entries.isZeroGain];
            included = entries(nonzeroMask);

            lines = { ...
                'Batch Identifiability Report'; ...
                'Zero adaptive gain runs are excluded.'; ...
                ''};

            if isempty(included)
                lines{end+1,1} = 'No nonzero adaptive-gain runs were found.';
                reportText = strjoin(lines, newline);
                return;
            end

            gainIds = unique([included.gainIndex], 'stable');
            for gainId = gainIds
                gainRows = included([included.gainIndex] == gainId);
                lines{end+1,1} = sprintf('Gain %03d', gainId);
                lines{end+1,1} = obj.formatGainVector(gainRows(1).gamma);
                headers = {'Trajectory', ...
                    'Mass Ident Metric', 'Mass Ident Score', ...
                    'CoG Ident Metric', 'CoG Ident Score', ...
                    'Inertia Ident Metric', 'Inertia Ident Score'};
                rows = cell(numel(gainRows), numel(headers));
                for i = 1:numel(gainRows)
                    rows{i,1} = gainRows(i).trajectory;
                    rows{i,2} = obj.fmtMetric(gainRows(i).massMetric, 4);
                    rows{i,3} = obj.fmtMetric(gainRows(i).massScore, 2);
                    rows{i,4} = obj.fmtMetric(gainRows(i).cogMetric, 4);
                    rows{i,5} = obj.fmtMetric(gainRows(i).cogScore, 2);
                    rows{i,6} = obj.fmtMetric(gainRows(i).inertiaMetric, 4);
                    rows{i,7} = obj.fmtMetric(gainRows(i).inertiaScore, 2);
                end
                tableLines = cellstr(obj.buildTable(headers, rows));
                lines = [lines; tableLines; {''}];
            end

            lines{end+1,1} = 'Trajectory Mean Summary';
            lines = [lines; cellstr(obj.buildIdentifiabilitySummary(included))];
            reportText = strjoin(lines, newline);
        end

        function entries = collectIdentifiabilityEntries(obj)
            %COLLECTIDENTIFIABILITYENTRIES Read batch metrics for identifiability reporting.
            entries = repmat(struct( ...
                'trajectory', '', ...
                'runLabel', '', ...
                'gainIndex', NaN, ...
                'gamma', [], ...
                'isZeroGain', false, ...
                'massMetric', NaN, ...
                'massScore', NaN, ...
                'cogMetric', NaN, ...
                'cogScore', NaN, ...
                'inertiaMetric', NaN, ...
                'inertiaScore', NaN), numel(obj.childDirs), 1);

            for i = 1:numel(obj.childDirs)
                metricsEntry = vt.sim.ResultsManager.loadMetricsFile(obj.childDirs{i});
                entries(i).trajectory = metricsEntry.trajectory;
                entries(i).runLabel = metricsEntry.run_label;
                entries(i).gainIndex = obj.readGainIndex(metricsEntry.run_label);
                entries(i).gamma = obj.gammaForGainIndex(entries(i).gainIndex);
                entries(i).isZeroGain = obj.isZeroAdaptiveGain(entries(i).gamma);
                entries(i).massMetric = obj.readStructField(metricsEntry, 'mass_ident_metric');
                entries(i).massScore = obj.readStructField(metricsEntry, 'mass_ident_score');
                entries(i).cogMetric = obj.readStructField(metricsEntry, 'cog_ident_metric');
                entries(i).cogScore = obj.readStructField(metricsEntry, 'cog_ident_score');
                entries(i).inertiaMetric = obj.readStructField(metricsEntry, 'inertia_ident_metric');
                entries(i).inertiaScore = obj.readStructField(metricsEntry, 'inertia_ident_score');
            end
        end

        function rowsText = buildIdentifiabilitySummary(obj, entries)
            %BUILDIDENTIFIABILITYSUMMARY Build trajectory-wise mean identifiability table.
            trajNames = unique({entries.trajectory}, 'stable');
            headers = {'Trajectory', ...
                'Mass Ident Metric Mean', 'Mass Ident Score Mean', ...
                'CoG Ident Metric Mean', 'CoG Ident Score Mean', ...
                'Inertia Ident Metric Mean', 'Inertia Ident Score Mean'};
            rows = cell(numel(trajNames), numel(headers));
            for i = 1:numel(trajNames)
                traj = trajNames{i};
                trajEntries = entries(strcmp({entries.trajectory}, traj));
                rows{i,1} = traj;
                rows{i,2} = obj.fmtMetric(obj.meanFinite([trajEntries.massMetric]), 4);
                rows{i,3} = obj.fmtMetric(obj.meanFinite([trajEntries.massScore]), 2);
                rows{i,4} = obj.fmtMetric(obj.meanFinite([trajEntries.cogMetric]), 4);
                rows{i,5} = obj.fmtMetric(obj.meanFinite([trajEntries.cogScore]), 2);
                rows{i,6} = obj.fmtMetric(obj.meanFinite([trajEntries.inertiaMetric]), 4);
                rows{i,7} = obj.fmtMetric(obj.meanFinite([trajEntries.inertiaScore]), 2);
            end
            rowsText = obj.buildTable(headers, rows);
        end

        function lines = buildTable(obj, headers, rows)
            %BUILDTABLE Build an aligned plain-text table.
            nRows = size(rows, 1);
            widths = cellfun(@strlength, headers);
            for col = 1:numel(headers)
                for row = 1:nRows
                    widths(col) = max(widths(col), strlength(string(rows{row,col})));
                end
            end

            lines = strings(nRows + 3, 1);
            lineIdx = 1;
            lines(lineIdx) = obj.buildSep(widths); lineIdx = lineIdx + 1;
            lines(lineIdx) = obj.buildRow(headers, widths); lineIdx = lineIdx + 1;
            lines(lineIdx) = obj.buildSep(widths); lineIdx = lineIdx + 1;
            for row = 1:nRows
                lines(lineIdx) = obj.buildRow(rows(row,:), widths);
                lineIdx = lineIdx + 1;
            end
            lines(lineIdx) = obj.buildSep(widths);
        end

        function gamma = gammaForGainIndex(obj, gainIndex)
            %GAMMAFORGAININDEX Reconstruct the Gamma row for a batch gain index.
            gamma = [];
            if ~isfield(obj.cfg.controller, 'Gamma') || isempty(obj.cfg.controller.Gamma) || ~isfinite(gainIndex)
                return;
            end
            gammaValue = obj.cfg.controller.Gamma;
            if isvector(gammaValue) || size(gammaValue, 1) == 1
                gamma = gammaValue(:);
            elseif gainIndex >= 1 && gainIndex <= size(gammaValue, 1)
                gamma = gammaValue(gainIndex, :).';
            end
        end

        function gainIndex = readGainIndex(~, runLabel)
            %READGAININDEX Parse the gain index from the run label.
            gainIndex = NaN;
            tokens = regexp(char(string(runLabel)), '^Run\s+(\d+)$', 'tokens', 'once');
            if ~isempty(tokens)
                gainIndex = str2double(tokens{1});
            end
        end

        function tf = isZeroAdaptiveGain(~, gamma)
            %ISZEROADAPTIVEGAIN Return true when all adaptive gains are zero.
            tf = ~isempty(gamma) && all(gamma == 0);
        end

        function text = formatGainVector(obj, gamma)
            %FORMATGAINVECTOR Format one Gamma vector for report output.
            if isempty(gamma)
                text = 'Gamma: N/A';
                return;
            end
            formatted = arrayfun(@(x) sprintf('%.4f', x), gamma(:).', 'UniformOutput', false);
            text = sprintf('Gamma: [%s]', strjoin(formatted, '  '));
        end

        function value = meanFinite(~, values)
            %MEANFINITE Return mean over finite values or NaN when unavailable.
            finiteMask = isfinite(values);
            if ~any(finiteMask)
                value = NaN;
                return;
            end
            value = mean(values(finiteMask));
        end

        function value = readStructField(~, s, fieldName)
            %READSTRUCTFIELD Read a scalar struct field with NaN fallback.
            value = NaN;
            if isfield(s, fieldName) && ~isempty(s.(fieldName))
                value = s.(fieldName);
            end
        end

        function text = fmtMetric(~, value, decimals)
            %FMTMETRIC Format a numeric metric with fixed decimals.
            if ~isfinite(value)
                text = 'N/A';
                return;
            end
            text = sprintf(['%0.' num2str(decimals) 'f'], value);
        end

        function line = buildRow(~, values, widths)
            %BUILDROW Build one padded plain-text table row.
            parts = cell(1, numel(values));
            for i = 1:numel(values)
                parts{i} = char(pad(string(values{i}), widths(i), 'right'));
            end
            line = sprintf('| %s |', strjoin(parts, ' | '));
        end

        function line = buildSep(~, widths)
            %BUILDSEP Build a horizontal separator for the table.
            parts = cell(1, numel(widths));
            for i = 1:numel(widths)
                parts{i} = repmat('-', 1, widths(i));
            end
            line = sprintf('+-%s-+', strjoin(parts, '-+-'));
        end
    end
end
