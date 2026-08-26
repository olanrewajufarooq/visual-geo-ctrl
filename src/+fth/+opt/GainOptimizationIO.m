classdef GainOptimizationIO
    %GAINOPTIMIZATIONIO Persist optimizer reports, scripts, and plots.
    methods (Static)
        function data = gainTable(iteration, cost, elapsed, x, event)
            %GAINTABLE Build a self-describing physical-gain history row.
            x = double(x(:).');
            if numel(x) < 18
                error('fth:GainOptimizationIO:InvalidGainVector', ...
                    'A gain vector must contain at least 18 controller variables.');
            end
            gamma = 10 .^ x(19:end);
            values = [{iteration}, {cost}, {elapsed}, {string(event)}];
            values = [values, num2cell(x(1:6)), num2cell(x(7:12)), ...
                num2cell(x(13:18)), num2cell(gamma)];
            data = cell2table(values, 'VariableNames', fth.opt.GainOptimizationIO.gainVariableNames(numel(gamma)));
        end

        function history = emptyGainHistory(dimension)
            %EMPTYGAINHISTORY Create an empty typed gain-history table.
            if dimension < 18 || floor(dimension) ~= dimension
                error('fth:GainOptimizationIO:InvalidDimension', ...
                    'Gain-history dimension must be an integer of at least 18.');
            end
            gammaCount = dimension - 18;
            names = fth.opt.GainOptimizationIO.gainVariableNames(gammaCount);
            history = table(zeros(0, 1), zeros(0, 1), zeros(0, 1), strings(0, 1), ...
                'VariableNames', names(1:4));
            for i = 5:numel(names)
                history.(names{i}) = zeros(0, 1);
            end
        end

        function history = appendImprovement(history, iteration, cost, elapsed, x)
            %APPENDIMPROVEMENT Append only a strict best-cost improvement.
            if isempty(history) || cost < history.best_cost(end)
                row = fth.opt.GainOptimizationIO.gainTable(iteration, cost, elapsed, x, 'improvement');
                if isempty(history)
                    history = row;
                else
                    history = [history; row]; %#ok<AGROW>
                end
            end
        end

        function writeSummary(folder, scenario, x, cost, elapsed, breakdown)
            fid = fopen(fullfile(folder, 'best_gains.txt'), 'w'); cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
            fprintf(fid, 'scenario = %s\nadaptation = %s\ncoriolis_form = %s\n', scenario.id, scenario.adaptation, scenario.coriolisForm);
            fprintf(fid, 'with_payload_drop = %d\nparam_init = %s\ncost = %.16g\nelapsed_seconds = %.6f\n', scenario.withPayload, scenario.paramInit, cost, elapsed);
            fprintf(fid, 'Kp = [%s]\nKd = [%s]\nlambda = [%s]\n', sprintf(' %.16g', x(1:6)), sprintf(' %.16g', x(7:12)), sprintf(' %.16g', x(13:18)));
            if numel(x) > 18, fprintf(fid, 'Gamma = [%s]\n', sprintf(' %.16g', 10.^x(19:end))); else, fprintf(fid, 'Gamma = []\n'); end
            fprintf(fid, 'scenario_failures = %d\n', sum([breakdown.scenario.failed]));
        end

        function writeScript(folder, scenario, x, cost)
            fid = fopen(fullfile(folder, 'best_gains.m'), 'w'); cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
            fprintf(fid, '%% Best gains from optimize_gains for %s\n', scenario.id);
            fprintf(fid, 'scenario_id = ''%s''; adaptation = ''%s''; coriolis_form = ''%s'';\n', scenario.id, scenario.adaptation, scenario.coriolisForm);
            fprintf(fid, 'param_init = ''%s''; with_payload_drop = %d; cost = %.16g;\n', scenario.paramInit, scenario.withPayload, cost);
            fprintf(fid, 'Kp = [%s].''; Kd = [%s].''; lambda = [%s].'';\n', sprintf(' %.16g', x(1:6)), sprintf(' %.16g', x(7:12)), sprintf(' %.16g', x(13:18)));
            if numel(x) > 18, fprintf(fid, 'Gamma = [%s].'';\n', sprintf(' %.16g', 10.^x(19:end))); else, fprintf(fid, 'Gamma = [];\n'); end
        end

        function plotConvergence(folder, history, scenarioId)
            if isempty(history), return; end
            valid = isfinite(history(:, 1)) & isfinite(history(:, 2));
            history = history(valid, :);
            if isempty(history), return; end
            fig = figure('Visible', 'off');
            plot(history(:, 1), history(:, 2), '-o', 'LineWidth', 1.5, ...
                'MarkerSize', 5, 'MarkerFaceColor', [0.1 0.35 0.8]);
            grid on; xlabel('Generation'); ylabel('Best cost'); title(['PSO convergence: ' strrep(scenarioId, '-', ' ')]);
            if size(history, 1) == 1
                xlim(history(1, 1) + [-0.5, 0.5]);
                y = history(1, 2);
                yPad = max(0.05 * max(abs(y), 1), 1e-3);
                ylim(y + [-yPad, yPad]);
            end
            saveas(fig, fullfile(folder, 'convergence.png')); close(fig);
        end

        function printBestGains(scenario, x, cost)
            %PRINTBESTGAINS Print the optimized vector for immediate reuse.
            fprintf('\n[%s] Optimal gains (cost %.6g)\n', scenario.id, cost);
            fprintf('  Kp:     [%s]\n', sprintf(' %.8g', x(1:6)));
            fprintf('  Kd:     [%s]\n', sprintf(' %.8g', x(7:12)));
            fprintf('  lambda: [%s]\n', sprintf(' %.8g', x(13:18)));
            if numel(x) > 18
                fprintf('  Gamma:  [%s]\n', sprintf(' %.8g', 10.^x(19:end)));
            else
                fprintf('  Gamma:  []\n');
            end
        end

        function writeConvergence(folder, history)
            %WRITECONVERGENCE Save PSO history with a self-describing header.
            data = array2table(history, 'VariableNames', ...
                {'iteration', 'best_cost', 'elapsed_seconds'});
            writetable(data, fullfile(folder, 'convergence.csv'));
        end

        function writeImprovements(folder, history)
            %WRITEIMPROVEMENTS Save physical gains at each best improvement.
            writetable(history, fullfile(folder, 'best_improvements.csv'));
        end

        function writeBaselineAndBest(folder, baselineVector, baselineBreakdown, bestX, bestCost)
            %WRITEBASELINEANDBEST Save named baseline and optimized gain rows.
            baselineCost = NaN;
            if isstruct(baselineBreakdown) && isfield(baselineBreakdown, 'cost')
                baselineCost = baselineBreakdown.cost;
            end
            baselineRow = fth.opt.GainOptimizationIO.gainTable(0, baselineCost, 0, ...
                baselineVector, 'baseline');
            bestRow = fth.opt.GainOptimizationIO.gainTable(NaN, bestCost, NaN, ...
                bestX, 'best');
            writetable([baselineRow; bestRow], fullfile(folder, 'baseline_and_best.csv'));
        end

        function writeMetadata(folder, scenario, opts, runSignature, exitflag, output, completed, elapsed)
            %WRITEMETADATA Save compact machine-readable run metadata.
            iterations = NaN;
            if isstruct(output) && isfield(output, 'iterations')
                iterations = output.iterations;
            end
            metadata = struct('scenario_id', scenario.id, ...
                'adaptation', scenario.adaptation, 'coriolis_form', scenario.coriolisForm, ...
                'with_payload_drop', scenario.withPayload, 'completed', completed, ...
                'exitflag', exitflag, 'iterations', iterations, ...
                'elapsed_seconds', elapsed, 'run_signature', runSignature, ...
                'options', opts);
            fid = fopen(fullfile(folder, 'run_metadata.json'), 'w');
            if fid < 0
                error('fth:GainOptimizationIO:MetadataWriteFailed', ...
                    'Could not open run_metadata.json for writing.');
            end
            cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
            fprintf(fid, '%s\n', jsonencode(metadata));
        end

        function signature = makeSignature(opts, scenario)
            %MAKESIGNATURE Identify every setting that changes an optimization result.
            fields = {'scenarios', 'outputRoot', 'cacheRoot', 'clearCache'};
            options = opts;
            for i = 1:numel(fields)
                if isfield(options, fields{i})
                    options = rmfield(options, fields{i});
                end
            end
            scenarioForSignature = scenario;
            if isfield(scenarioForSignature, 'clearCache')
                scenarioForSignature.clearCache = [];
            end
            signature = jsonencode(struct('scenario', scenarioForSignature, ...
                'options', options));
        end

        function [state, folder, found] = findState(cacheRoot, scenarioId, signature)
            %FINDSTATE Find the matching state in the central scenario cache.
            state = struct();
            folder = fullfile(cacheRoot, scenarioId);
            found = false;
            stateFile = fullfile(folder, 'optimizer_state.mat');
            if ~isfile(stateFile)
                return;
            end
            try
                candidate = load(stateFile);
                if isfield(candidate, 'runSignature') && ...
                        strcmp(candidate.runSignature, signature)
                    state = candidate;
                    found = true;
                end
            catch
                % Ignore incomplete or incompatible cached state.
            end
        end

        function clearState(cacheRoot, scenarioId)
            %CLEARSTATE Remove only one scenario's central cache.
            folder = fullfile(cacheRoot, scenarioId);
            if isfolder(folder)
                rmdir(folder, 's');
            end
        end

        function bestX = loadBestVector(reportRoot, scenarioId, expectedDimension, expectedMode)
            %LOADBESTVECTOR Load and validate a completed general-run result.
            stateFile = fullfile(reportRoot, scenarioId, 'optimizer_state.mat');
            if ~isfile(stateFile)
                error('fth:GainOptimizationIO:MissingSource', ...
                    'Source report is missing: %s.', stateFile);
            end
            try
                state = load(stateFile);
            catch err
                error('fth:GainOptimizationIO:InvalidSource', ...
                    'Could not load source report %s: %s', stateFile, err.message);
            end
            if ~isfield(state, 'completed') || ~state.completed || ...
                    ~isfield(state, 'bestX')
                error('fth:GainOptimizationIO:IncompleteSource', ...
                    'Source report is not a completed optimizer state: %s.', stateFile);
            end
            bestX = double(state.bestX(:).');
            if numel(bestX) ~= expectedDimension
                error('fth:GainOptimizationIO:IncompatibleSource', ...
                    'Expected %d gains for %s, found %d.', expectedDimension, scenarioId, numel(bestX));
            end
            if isfield(state, 'scenario') && isfield(state.scenario, 'adaptation') && ...
                    ~strcmpi(state.scenario.adaptation, expectedMode)
                error('fth:GainOptimizationIO:IncompatibleSource', ...
                    'Source report adaptation does not match expected mode %s.', expectedMode);
            end
        end

        function names = gainVariableNames(gammaCount)
            names = [{'iteration', 'best_cost', 'elapsed_seconds', 'event'}, ...
                arrayfun(@(i) sprintf('Kp_%d', i), 1:6, 'UniformOutput', false), ...
                arrayfun(@(i) sprintf('Kd_%d', i), 1:6, 'UniformOutput', false), ...
                arrayfun(@(i) sprintf('lambda_%d', i), 1:6, 'UniformOutput', false), ...
                arrayfun(@(i) sprintf('Gamma_%d', i), 1:gammaCount, 'UniformOutput', false)];
        end
    end
end
