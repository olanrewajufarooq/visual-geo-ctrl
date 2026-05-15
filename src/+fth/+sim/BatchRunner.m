classdef BatchRunner < handle
    %BATCHRUNNER Orchestrates multi-trajectory and multi-gain batch runs.
    %   Manages child SimRunner creation, per-run console capture, aggregate
    %   report generation, and batch plot dispatch.
    %
    %   Usage:
    %     br = fth.sim.BatchRunner(cfg, resultsDir, batchSize);
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
            %     cfg - fth.sim.Config instance.
            %     resultsDir - root results directory for this batch.
            %     batchSize - total number of runs.
            obj.cfg = cfg;
            obj.resultsDir = resultsDir;
            obj.batchSize = batchSize;
            obj.childDirs = {};
            obj.console = fth.io.ConsoleCapture();
        end

        function runAll(obj, runArgs)
            %RUNALL Execute all batch runs with per-run console capture.
            %   Input: runArgs - cell array of arguments for child.run().
            cfgs = obj.cfg.expandBatchConfigs(obj.resultsDir);
            obj.childDirs = cell(obj.batchSize, 1);
            failed = cell(obj.batchSize, 1);
            failedCount = 0;
            for i = 1:obj.batchSize
                child = fth.sim.SimRunner(cfgs{i});
                runLog = obj.console.capture(@() obj.executeChild(child, runArgs));
                childLogPath = fullfile(child.resultsDir, 'command_window.txt');
                fth.io.ResultsManager.writeTextFile(childLogPath, strtrim(runLog));
                obj.childDirs{i} = child.resultsDir;
                if contains(runLog, 'Error using') || contains(runLog, 'Error in') || ...
                        contains(runLog, 'Unrecognized') || contains(runLog, 'Undefined')
                    failedCount = failedCount + 1;
                    failed{failedCount} = child.resultsDir;
                end
                clear child
            end
            obj.writeAggregateArtifacts();
            fprintf('Batch results saved to: %s\n', obj.resultsDir);
            if failedCount > 0
                warning('fth:BatchRunner:childFailed', ...
                    '%d/%d runs failed. Check command_window.txt in:\n%s', ...
                    failedCount, obj.batchSize, strjoin(failed(1:failedCount), '\n'));
            end
        end

        function plotAll(obj, plotType, displayPlots)
            %PLOTALL Generate plots for each saved batch run.
            if nargin < 3 || isempty(displayPlots)
                displayPlots = false;
            end
            if isempty(obj.childDirs)
                obj.childDirs = fth.io.ResultsManager.findChildResultDirs(obj.resultsDir);
            end
            if isempty(obj.childDirs)
                error('BatchRunner:NotRun', 'Batch simulation has not been run yet.');
            end
            for i = 1:numel(obj.childDirs)
                fth.io.ResultsManager.plotSavedRun(obj.childDirs{i}, char(plotType), displayPlots);
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
                obj.childDirs = fth.io.ResultsManager.findChildResultDirs(obj.resultsDir);
            end
            aggregateChunks = cell(numel(obj.childDirs), 1);
            for i = 1:numel(obj.childDirs)
                metricsEntry = fth.io.ResultsManager.loadMetricsFile(obj.childDirs{i});
                childLogPath = fullfile(obj.childDirs{i}, 'command_window.txt');
                childLog = fth.io.ResultsManager.readTextFile(childLogPath);
                aggregateChunks{i} = sprintf('%s%s\n', ...
                    fth.io.ConsoleFormatter.runBanner( ...
                    metricsEntry.trajectory, metricsEntry.run_label, metricsEntry.is_adaptive), ...
                    strtrim(childLog));
            end
            aggregatePath = fullfile(obj.resultsDir, 'command_window.txt');
            fth.io.ResultsManager.writeTextFile(aggregatePath, strjoin(aggregateChunks, newline));
            if obj.isAdaptiveBatch()
                summaryPath = fullfile(obj.resultsDir, 'adaptive_report.txt');
                fth.io.ResultsManager.writeTextFile(summaryPath, fth.sim.BatchRunnerUtils.buildSummaryTable(obj.childDirs));
                identPath = fullfile(obj.resultsDir, 'regressor_info_report.txt');
                fth.io.ResultsManager.writeTextFile(identPath, fth.sim.BatchRunnerUtils.buildRegressorInfoReport(obj.childDirs, obj.cfg));
            end
        end

        function tf = isAdaptiveBatch(obj)
            %ISADAPTIVEBATCH Return true when all saved batch runs use adaptation.
            tf = ~isempty(obj.childDirs);
            if ~tf, return; end
            for i = 1:numel(obj.childDirs)
                metricsEntry = fth.io.ResultsManager.loadMetricsFile(obj.childDirs{i});
                if ~isfield(metricsEntry, 'is_adaptive') || ~metricsEntry.is_adaptive
                    tf = false;
                    return;
                end
            end
        end

    end
end
