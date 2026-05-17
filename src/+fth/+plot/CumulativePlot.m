classdef CumulativePlot
    %CUMULATIVEPLOT Static utilities for across-run batch visualizations.
    %   Generates grouped bar charts that compare metrics across all N runs
    %   for a single trajectory directory.
    %
    %   Usage (called by BatchRunner after all runs complete):
    %     fth.plot.CumulativePlot.plot(outDir, entries, runLabels, modes)

    methods (Static)
        function plot(outDir, entries, runLabels, modes)
            %PLOT Dispatcher — generate each requested cumulative plot.
            %   Inputs:
            %     outDir     - directory where PNGs are saved.
            %     entries    - N×1 cell of metricsEntry structs (from loadMetricsFile).
            %     runLabels  - N×1 cell of run label strings.
            %     modes      - cell of mode strings: 'spd', 'tracking_rmse',
            %                  'estimation_nrmse'.
            for k = 1:numel(modes)
                switch lower(char(modes{k}))
                    case 'spd'
                        fth.plot.CumulativePlot.plotSPD(outDir, entries, runLabels);
                    case 'tracking_rmse'
                        fth.plot.CumulativePlot.plotTrackingRMSE(outDir, entries, runLabels);
                    case 'estimation_nrmse'
                        fth.plot.CumulativePlot.plotEstimationNRMSE(outDir, entries, runLabels);
                end
            end
        end

        function plotSPD(outDir, entries, runLabels)
            %PLOTSPD Grouped bar chart of SPD valid/invalid percentages across N runs.
            %   Left bar = valid (blue), right bar = invalid (red).
            N = numel(entries);
            data = zeros(N, 2);
            for i = 1:N
                m = entries{i};
                valid   = 0;
                invalid = 0;
                if isfield(m, 'spd_valid_count') && isfinite(m.spd_valid_count)
                    valid = m.spd_valid_count;
                end
                if isfield(m, 'spd_invalid_count') && isfinite(m.spd_invalid_count)
                    invalid = m.spd_invalid_count;
                end
                total = valid + invalid;
                if total > 0
                    data(i, 1) = 100 * valid   / total;
                    data(i, 2) = 100 * invalid / total;
                end
            end

            fig = figure('Name', 'Cumulative SPD Validity', 'Visible', 'off', ...
                'Position', [100 100 800 420]);
            ax = axes(fig);
            b = bar(ax, data, 'grouped');
            b(1).FaceColor = [0 0 1];
            b(2).FaceColor = [1 0 0];
            ax.XTick = 1:N;
            ax.XTickLabel = runLabels;
            ax.XTickLabelRotation = 30;
            ylim(ax, [0 100]);
            ylabel(ax, 'Percentage [%]');
            legend(ax, {'Valid', 'Invalid'}, 'Location', 'best');
            title(ax, 'SPD Validity Across Runs');
            grid(ax, 'on');
            saveas(fig, fullfile(outDir, 'cumulative_spd.png'));
            close(fig);
        end

        function plotTrackingRMSE(outDir, entries, runLabels)
            %PLOTTRACKINGRMSE Bar chart of tracking RMSE across N runs.
            N = numel(entries);
            data = nan(N, 1);
            for i = 1:N
                m = entries{i};
                if isfield(m, 'track_rmse') && isfinite(m.track_rmse)
                    data(i) = m.track_rmse;
                end
            end

            fig = figure('Name', 'Cumulative Tracking RMSE', 'Visible', 'off', ...
                'Position', [100 100 800 420]);
            ax = axes(fig);
            bar(ax, data);
            ax.XTick = 1:N;
            ax.XTickLabel = runLabels;
            ax.XTickLabelRotation = 30;
            ylabel(ax, 'RMSE [m]');
            title(ax, 'Tracking RMSE Across Runs');
            grid(ax, 'on');
            saveas(fig, fullfile(outDir, 'cumulative_tracking_rmse.png'));
            close(fig);
        end

        function plotEstimationNRMSE(outDir, entries, runLabels)
            %PLOTESTIMATIONNRMSE Grouped bar chart of estimation NRMSE across N runs.
            %   mass=red, cog=green, inertia=blue.
            N = numel(entries);
            data = nan(N, 3);
            for i = 1:N
                m = entries{i};
                if isfield(m, 'mass_nrmse') && isfinite(m.mass_nrmse)
                    data(i, 1) = m.mass_nrmse;
                end
                if isfield(m, 'cog_nrmse') && isfinite(m.cog_nrmse)
                    data(i, 2) = m.cog_nrmse;
                end
                if isfield(m, 'inertia_nrmse') && isfinite(m.inertia_nrmse)
                    data(i, 3) = m.inertia_nrmse;
                end
            end

            fig = figure('Name', 'Cumulative Estimation NRMSE', 'Visible', 'off', ...
                'Position', [100 100 800 420]);
            ax = axes(fig);
            b = bar(ax, data, 'grouped');
            b(1).FaceColor = [1 0 0];
            b(2).FaceColor = [0 1 0];
            b(3).FaceColor = [0 0 1];
            ax.XTick = 1:N;
            ax.XTickLabel = runLabels;
            ax.XTickLabelRotation = 30;
            ylabel(ax, 'Normalized RMSE');
            legend(ax, {'Mass', 'CoG', 'Inertia'}, 'Location', 'best');
            title(ax, 'Estimation NRMSE Across Runs');
            grid(ax, 'on');
            saveas(fig, fullfile(outDir, 'cumulative_estimation_nrmse.png'));
            close(fig);
        end
    end
end
