classdef GainOptimizationIO
    %GAINOPTIMIZATIONIO Persist optimizer reports, scripts, and plots.
    methods (Static)
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
    end
end
