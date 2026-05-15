classdef BatchRunnerUtils
    %BATCHRUNNERUTILS Static utility helpers for fth.sim.BatchRunner.
    %   All methods are static — no instance needed.

    methods (Static)

        function tableText = buildSummaryTable(childDirs)
            %BUILDSUMMARYTABLE Build an aligned summary table from saved runs.
            nRuns = numel(childDirs);
            headers = {'Trajectory', 'Run', 'Track RMSE', ...
                'Mass RMSE', 'Mass NRMSE', 'Mass Regressor Info', ...
                'CoG RMSE', 'CoG NRMSE', 'CoG Regressor Info', ...
                'Inertia RMSE', 'Inertia NRMSE', 'Inertia Regressor Info'};
            rawRows = cell(nRuns, numel(headers));
            numericValues = nan(nRuns, numel(headers));
            trajectoryNames = cell(nRuns, 1);
            betterIsLower = [false, false, true, ...
                true, true, false, ...
                true, true, false, ...
                true, true, false];

            for i = 1:nRuns
                metrics = fth.io.ResultsManager.loadMetricsFile(childDirs{i});
                trajectoryNames{i} = metrics.trajectory;
                rawRows{i,1} = trajectoryNames{i};
                rawRows{i,2} = metrics.run_label;

                rawRows{i,3} = fth.sim.BatchRunnerUtils.fmtMetric(metrics.track_rmse, 4);
                numericValues(i,3) = metrics.track_rmse;

                if metrics.is_adaptive
                    rawRows{i,4} = fth.sim.BatchRunnerUtils.fmtMetric(metrics.mass_rmse, 4);
                    rawRows{i,5} = fth.sim.BatchRunnerUtils.fmtMetric(metrics.mass_nrmse, 4);
                    rawRows{i,6} = fth.sim.BatchRunnerUtils.fmtMetric(metrics.mass_regressor_info, 4);
                    rawRows{i,7} = fth.sim.BatchRunnerUtils.fmtMetric(metrics.cog_rmse, 4);
                    rawRows{i,8} = fth.sim.BatchRunnerUtils.fmtMetric(metrics.cog_nrmse, 4);
                    rawRows{i,9} = fth.sim.BatchRunnerUtils.fmtMetric(metrics.cog_regressor_info, 4);
                    rawRows{i,10} = fth.sim.BatchRunnerUtils.fmtMetric(metrics.inertia_rmse, 4);
                    rawRows{i,11} = fth.sim.BatchRunnerUtils.fmtMetric(metrics.inertia_nrmse, 4);
                    rawRows{i,12} = fth.sim.BatchRunnerUtils.fmtMetric(metrics.inertia_regressor_info, 4);
                    numericValues(i,4) = metrics.mass_rmse;
                    numericValues(i,5) = metrics.mass_nrmse;
                    numericValues(i,6) = metrics.mass_regressor_info;
                    numericValues(i,7) = metrics.cog_rmse;
                    numericValues(i,8) = metrics.cog_nrmse;
                    numericValues(i,9) = metrics.cog_regressor_info;
                    numericValues(i,10) = metrics.inertia_rmse;
                    numericValues(i,11) = metrics.inertia_nrmse;
                    numericValues(i,12) = metrics.inertia_regressor_info;
                else
                    rawRows(i,4:12) = {'N/A'};
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
            lines(lineIdx) = fth.sim.BatchRunnerUtils.buildSep(widths); lineIdx = lineIdx + 1;
            lines(lineIdx) = fth.sim.BatchRunnerUtils.buildRow(headers, widths); lineIdx = lineIdx + 1;
            lines(lineIdx) = fth.sim.BatchRunnerUtils.buildSep(widths); lineIdx = lineIdx + 1;
            for row = 1:nRuns
                lines(lineIdx) = fth.sim.BatchRunnerUtils.buildRow(rawRows(row,:), widths);
                lineIdx = lineIdx + 1;
            end
            lines(lineIdx) = fth.sim.BatchRunnerUtils.buildSep(widths);
            tableText = strjoin(cellstr(lines), newline);
        end

        function reportText = buildRegressorInfoReport(childDirs, cfg)
            %BUILDREGRESSORINFOREPORT Build a gain-first regressor information report.
            entries = fth.sim.BatchRunnerUtils.collectRegressorInfoEntries(childDirs, cfg);
            nonzeroMask = ~[entries.isZeroGain];
            included = entries(nonzeroMask);

            lines = { ...
                'Batch Regressor Information Report'; ...
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
                lines{end+1,1} = fth.sim.BatchRunnerUtils.formatGainVector(gainRows(1).gamma);
                headers = {'Trajectory', ...
                    'Mass Regressor Info', ...
                    'CoG Regressor Info', ...
                    'Inertia Regressor Info'};
                rows = cell(numel(gainRows), numel(headers));
                for i = 1:numel(gainRows)
                    rows{i,1} = gainRows(i).trajectory;
                    rows{i,2} = fth.sim.BatchRunnerUtils.fmtMetric(gainRows(i).massMetric, 4);
                    rows{i,3} = fth.sim.BatchRunnerUtils.fmtMetric(gainRows(i).cogMetric, 4);
                    rows{i,4} = fth.sim.BatchRunnerUtils.fmtMetric(gainRows(i).inertiaMetric, 4);
                end
                tableLines = cellstr(fth.sim.BatchRunnerUtils.buildTable(headers, rows));
                lines = [lines; tableLines; {''}];
            end

            lines{end+1,1} = 'Trajectory Mean Regressor Info Summary';
            lines = [lines; cellstr(fth.sim.BatchRunnerUtils.buildRegressorInfoSummary(included))];
            reportText = strjoin(lines, newline);
        end

        function rowsText = buildRegressorInfoSummary(entries)
            %BUILDREGRESSORINFOSUMMARY Build trajectory-wise mean regressor info table.
            trajNames = unique({entries.trajectory}, 'stable');
            headers = {'Trajectory', ...
                'Mass Regressor Info Mean', ...
                'CoG Regressor Info Mean', ...
                'Inertia Regressor Info Mean'};
            rows = cell(numel(trajNames), numel(headers));
            for i = 1:numel(trajNames)
                traj = trajNames{i};
                trajEntries = entries(strcmp({entries.trajectory}, traj));
                rows{i,1} = traj;
                rows{i,2} = fth.sim.BatchRunnerUtils.fmtMetric(fth.sim.BatchRunnerUtils.meanFinite([trajEntries.massMetric]), 4);
                rows{i,3} = fth.sim.BatchRunnerUtils.fmtMetric(fth.sim.BatchRunnerUtils.meanFinite([trajEntries.cogMetric]), 4);
                rows{i,4} = fth.sim.BatchRunnerUtils.fmtMetric(fth.sim.BatchRunnerUtils.meanFinite([trajEntries.inertiaMetric]), 4);
            end
            rowsText = fth.sim.BatchRunnerUtils.buildTable(headers, rows);
        end

        function entries = collectRegressorInfoEntries(childDirs, cfg)
            %COLLECTREGRESSORINFOENTRIES Read batch metrics for regressor info reporting.
            entries = repmat(struct( ...
                'trajectory', '', ...
                'runLabel', '', ...
                'gainIndex', NaN, ...
                'gamma', [], ...
                'isZeroGain', false, ...
                'massMetric', NaN, ...
                'cogMetric', NaN, ...
                'inertiaMetric', NaN), numel(childDirs), 1);

            for i = 1:numel(childDirs)
                metricsEntry = fth.io.ResultsManager.loadMetricsFile(childDirs{i});
                entries(i).trajectory = metricsEntry.trajectory;
                entries(i).runLabel = metricsEntry.run_label;
                % Prefer batch_run_index (written since Phase 2) over the regex fallback.
                if isfield(metricsEntry, 'batch_run_index') && isfinite(metricsEntry.batch_run_index)
                    entries(i).gainIndex = metricsEntry.batch_run_index;
                else
                    entries(i).gainIndex = fth.sim.BatchRunnerUtils.readGainIndex(metricsEntry.run_label);
                end
                entries(i).gamma = fth.sim.BatchRunnerUtils.gammaForGainIndex(cfg, entries(i).gainIndex);
                entries(i).isZeroGain = fth.sim.BatchRunnerUtils.isZeroAdaptiveGain(entries(i).gamma);
                entries(i).massMetric = fth.sim.BatchRunnerUtils.readStructField(metricsEntry, 'mass_regressor_info');
                entries(i).cogMetric = fth.sim.BatchRunnerUtils.readStructField(metricsEntry, 'cog_regressor_info');
                entries(i).inertiaMetric = fth.sim.BatchRunnerUtils.readStructField(metricsEntry, 'inertia_regressor_info');
            end
        end

        function lines = buildTable(headers, rows)
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
            lines(lineIdx) = fth.sim.BatchRunnerUtils.buildSep(widths); lineIdx = lineIdx + 1;
            lines(lineIdx) = fth.sim.BatchRunnerUtils.buildRow(headers, widths); lineIdx = lineIdx + 1;
            lines(lineIdx) = fth.sim.BatchRunnerUtils.buildSep(widths); lineIdx = lineIdx + 1;
            for row = 1:nRows
                lines(lineIdx) = fth.sim.BatchRunnerUtils.buildRow(rows(row,:), widths);
                lineIdx = lineIdx + 1;
            end
            lines(lineIdx) = fth.sim.BatchRunnerUtils.buildSep(widths);
        end

        function line = buildRow(values, widths)
            %BUILDROW Build one padded plain-text table row.
            parts = cell(1, numel(values));
            for i = 1:numel(values)
                parts{i} = char(pad(string(values{i}), widths(i), 'right'));
            end
            line = sprintf('| %s |', strjoin(parts, ' | '));
        end

        function line = buildSep(widths)
            %BUILDSEP Build a horizontal separator for the table.
            parts = cell(1, numel(widths));
            for i = 1:numel(widths)
                parts{i} = repmat('-', 1, widths(i));
            end
            line = sprintf('+-%s-+', strjoin(parts, '-+-'));
        end

        function gamma = gammaForGainIndex(cfg, gainIndex)
            %GAMMAFORGAININDEX Reconstruct the Gamma row for a batch gain index.
            gamma = [];
            if ~isfield(cfg.controller, 'Gamma') || isempty(cfg.controller.Gamma) || ~isfinite(gainIndex)
                return;
            end
            gammaValue = cfg.controller.Gamma;
            if isvector(gammaValue) || size(gammaValue, 1) == 1
                gamma = gammaValue(:);
            elseif gainIndex >= 1 && gainIndex <= size(gammaValue, 1)
                gamma = gammaValue(gainIndex, :).';
            end
        end

        function gainIndex = readGainIndex(runLabel)
            %READGAININDEX Parse the gain index from the run label.
            gainIndex = NaN;
            tokens = regexp(char(string(runLabel)), '^Run\s+(\d+)$', 'tokens', 'once');
            if ~isempty(tokens)
                gainIndex = str2double(tokens{1});
            end
        end

        function tf = isZeroAdaptiveGain(gamma)
            %ISZEROADAPTIVEGAIN Return true when all adaptive gains are zero.
            tf = ~isempty(gamma) && all(gamma == 0);
        end

        function text = formatGainVector(gamma)
            %FORMATGAINVECTOR Format one Gamma vector for report output.
            if isempty(gamma)
                text = 'Gamma: N/A';
                return;
            end
            formatted = arrayfun(@(x) sprintf('%.4f', x), gamma(:).', 'UniformOutput', false);
            text = sprintf('Gamma: [%s]', strjoin(formatted, '  '));
        end

        function text = fmtMetric(value, decimals)
            %FMTMETRIC Format a numeric metric with fixed decimals.
            if ~isfinite(value)
                text = 'N/A';
                return;
            end
            text = sprintf(['%0.' num2str(decimals) 'f'], value);
        end

        function value = meanFinite(values)
            %MEANFINITE Return mean over finite values or NaN when unavailable.
            finiteMask = isfinite(values);
            if ~any(finiteMask)
                value = NaN;
                return;
            end
            value = mean(values(finiteMask));
        end

        function value = readStructField(s, fieldName)
            %READSTRUCTFIELD Read a scalar struct field with NaN fallback.
            value = NaN;
            if isfield(s, fieldName) && ~isempty(s.(fieldName))
                value = s.(fieldName);
            end
        end

    end
end
