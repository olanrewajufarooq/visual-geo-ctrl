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
        console        % ConsoleCapture for sequential path
        useParallel    % true when PCT is available and cfg.sim.parallelRuns is set
    end

    methods
        function obj = BatchRunner(cfg, resultsDir, batchSize)
            %BATCHRUNNER Create a batch runner.
            %   Inputs:
            %     cfg - fth.sim.Config instance.
            %     resultsDir - root results directory for this batch.
            %     batchSize - total number of runs.
            obj.cfg         = cfg;
            obj.resultsDir  = resultsDir;
            obj.batchSize   = batchSize;
            obj.childDirs   = {};
            obj.console     = fth.io.ConsoleCapture();
            obj.useParallel = isfield(cfg.sim, 'parallelRuns') && cfg.sim.parallelRuns ...
                              && ~isempty(ver('parallel'));
        end

        function runAll(obj, runArgs)
            %RUNALL Execute all batch runs, in parallel or sequentially.
            %   Input: runArgs - cell array of arguments for child.run().
            cfgs = obj.cfg.expandBatchConfigs(obj.resultsDir);
            obj.childDirs = cell(obj.batchSize, 1);

            if obj.useParallel
                obj.runAllParallel(cfgs, runArgs);
            else
                obj.runAllSequential(cfgs, runArgs);
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
                obj.childDirs = fth.io.ResultsManager.findChildResultDirs(obj.resultsDir);
            end
            if isempty(obj.childDirs)
                error('BatchRunner:NotRun', 'Batch simulation has not been run yet.');
            end

            dirs = obj.childDirs;
            pt   = char(plotType);
            if obj.useParallel
                parfor i = 1:numel(dirs)
                    fth.io.ResultsManager.plotSavedRun(dirs{i}, pt, false);
                end
            else
                for i = 1:numel(dirs)
                    fth.io.ResultsManager.plotSavedRun(dirs{i}, pt, displayPlots);
                end
            end
            obj.writeAggregateArtifacts();
        end

        function dirs = getChildDirs(obj)
            %GETCHILDDIRS Return the list of child result directories.
            dirs = obj.childDirs;
        end
    end

    methods (Access = private)
        function runAllSequential(obj, cfgs, runArgs)
            %RUNALLSEQUENTIAL Execute runs one at a time with console capture.
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
            if failedCount > 0
                warning('fth:BatchRunner:childFailed', ...
                    '%d/%d runs failed. Check command_window.txt in:\n%s', ...
                    failedCount, obj.batchSize, strjoin(failed(1:failedCount), '\n'));
            end
        end

        function runAllParallel(obj, cfgs, runArgs)
            %RUNALLPARALLEL Execute runs via parfor; errors captured via try/catch.
            %   Console output from workers is not captured (acceptable trade-off).
            n = obj.batchSize;
            childDirs = cell(n, 1);
            childLogs = cell(n, 1);

            parfor i = 1:n
                child = fth.sim.SimRunner(cfgs{i});
                try
                    child.setup();
                    child.run(runArgs{:});
                    childLogs{i} = '';
                catch e
                    childLogs{i} = sprintf('Error: %s\nAt: %s line %d\n', ...
                        e.message, e.stack(1).name, e.stack(1).line);
                end
                childDirs{i} = child.resultsDir;
            end

            obj.childDirs = childDirs;
            failedCount = 0;
            failed = {};
            for i = 1:n
                logPath = fullfile(childDirs{i}, 'command_window.txt');
                fth.io.ResultsManager.writeTextFile(logPath, childLogs{i});
                if ~isempty(childLogs{i})
                    failedCount = failedCount + 1;
                    failed{end+1} = childDirs{i}; %#ok<AGROW>
                end
            end
            if failedCount > 0
                warning('fth:BatchRunner:childFailed', ...
                    '%d/%d runs failed. Check command_window.txt in:\n%s', ...
                    failedCount, n, strjoin(failed, '\n'));
            end
        end

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
