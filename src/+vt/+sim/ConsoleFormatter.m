classdef ConsoleFormatter
    %CONSOLEFORMATTER Shared plain-text formatting helpers for console output.
    %   Produces readable ASCII output for both MATLAB terminal display and
    %   saved command_window.txt files.

    methods (Static)
        function text = section(title)
            %SECTION Build a top-level section header.
            line = repmat('=', 1, 72);
            text = sprintf('\n%s\n%s\n%s\n', line, upper(char(string(title))), line);
        end

        function text = subsection(title)
            %SUBSECTION Build a second-level section header.
            line = repmat('-', 1, 56);
            text = sprintf('%s\n%s\n', char(string(title)), line);
        end

        function text = kv(label, value)
            %KV Build one aligned key/value line.
            text = sprintf('  %-18s %s\n', [char(string(label)) ':'], char(string(value)));
        end

        function text = note(message)
            %NOTE Build a note line with indentation.
            text = sprintf('  Note: %s\n', char(string(message)));
        end

        function text = vector(label, values, fmt, unit)
            %VECTOR Build a readable multi-line numeric vector line.
            if nargin < 3 || isempty(fmt)
                fmt = '%.4f';
            end
            if nargin < 4
                unit = '';
            end
            values = values(:).';
            formatted = arrayfun(@(x) sprintf(fmt, x), values, 'UniformOutput', false);
            text = sprintf('  %-18s [%s]', [char(string(label)) ':'], strjoin(formatted, '  '));
            if ~isempty(unit)
                text = sprintf('%s %s', text, unit);
            end
            text = sprintf('%s\n', text);
        end

        function text = timing(simDt, controlDt, adaptationDt, showAdaptation)
            %TIMING Build simulation timing block.
            text = '';
            text = [text, vt.sim.ConsoleFormatter.kv('sim_dt', sprintf('%.4f s', simDt))];
            text = [text, vt.sim.ConsoleFormatter.kv('control_dt', sprintf('%.4f s', controlDt))];
            if nargin >= 4 && showAdaptation
                text = [text, vt.sim.ConsoleFormatter.kv('adaptation_dt', sprintf('%.4f s', adaptationDt))];
            end
        end

        function text = runBanner(trajectory, runLabel, isAdaptive)
            %RUNBANNER Build a batch aggregate run banner.
            line = repmat('=', 1, 72);
            modeText = 'Nominal';
            if nargin >= 3 && isAdaptive
                modeText = 'Adaptive';
            end
            text = sprintf('\n%s\nRUN: %s | TRAJECTORY: %s | MODE: %s\n%s\n', ...
                line, char(string(runLabel)), char(string(trajectory)), modeText, line);
        end

        function text = headline(metrics, isAdaptive)
            %HEADLINE Build a one-line recap after the full metrics report.
            parts = { ...
                sprintf('Track RMSE %.4f', metrics.combined.rmse_total), ...
                sprintf('Track Score %.2f%%', metrics.combined.tracking_score)};
            if nargin >= 2 && isAdaptive && isfield(metrics, 'parameters')
                parts{end+1} = sprintf('Mass Tracking Score %.2f%%', metrics.parameters.mass.tracking_score);
                parts{end+1} = sprintf('CoG Tracking Score %.2f%%', metrics.parameters.cog.tracking_score);
                parts{end+1} = sprintf('Inertia Tracking Score %.2f%%', metrics.parameters.inertia.tracking_score);
            end
            text = sprintf('Summary: %s\n', strjoin(parts, ' | '));
        end
    end
end
