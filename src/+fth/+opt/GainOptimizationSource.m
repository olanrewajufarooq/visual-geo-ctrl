classdef GainOptimizationSource
    %GAINOPTIMIZATIONSOURCE Resolve fixed gains for secondary optimizations.
    methods (Static)
        function x = gainsToVector(gains, mode)
            %GAINSTOVECTOR Convert user-facing physical gains to optimizer space.
            required = {'Kp', 'Kd', 'lambda', 'Gamma'};
            if ~isstruct(gains) || ~all(isfield(gains, required))
                error('fth:GainOptimizationSource:InvalidGains', ...
                    'fixedGains must contain Kp, Kd, lambda, and Gamma.');
            end
            kp = double(gains.Kp(:).'); kd = double(gains.Kd(:).');
            lambda = double(gains.lambda(:).'); gamma = double(gains.Gamma(:).');
            expectedGamma = fth.opt.GainOptimizationUtils.gammaWidth(mode);
            if numel(kp) ~= 6 || numel(kd) ~= 6 || numel(lambda) ~= 6 || ...
                    numel(gamma) ~= expectedGamma || any(~isfinite([kp, kd, lambda, gamma])) || ...
                    any(gamma <= 0)
                error('fth:GainOptimizationSource:InvalidGains', ...
                    'fixedGains dimensions or values do not match %s.', mode);
            end
            x = [kp, kd, lambda, log10(gamma)];
        end

        function x = loadVector(sourceReportDir, searchRoot, scenarioId, expectedDimension, mode)
            %LOADVECTOR Load an explicit source or the newest general-run source.
            if nargin < 2 || isempty(searchRoot)
                searchRoot = fullfile('results', 'tuning');
            end
            if nargin < 1 || isempty(sourceReportDir)
                candidates = dir(fullfile(searchRoot, '*', scenarioId, 'optimizer_state.mat'));
                candidates = candidates(~contains(lower({candidates.folder}), ...
                    [filesep 'cache' filesep]));
                if isempty(candidates)
                    error('fth:GainOptimizationSource:MissingSource', ...
                        'No completed source report found for %s under %s.', scenarioId, searchRoot);
                end
                [~, order] = sort([candidates.datenum], 'descend');
                stateFile = fullfile(candidates(order(1)).folder, candidates(order(1)).name);
            else
                stateFile = fullfile(sourceReportDir, scenarioId, 'optimizer_state.mat');
            end
            state = load(stateFile);
            if ~isfield(state, 'completed') || ~state.completed || ~isfield(state, 'bestX')
                error('fth:GainOptimizationSource:IncompleteSource', ...
                    'Source report is incomplete: %s.', stateFile);
            end
            x = double(state.bestX(:).');
            if numel(x) ~= expectedDimension || ~isfield(state, 'scenario') || ...
                    ~strcmpi(state.scenario.adaptation, mode)
                error('fth:GainOptimizationSource:IncompatibleSource', ...
                    'Source report is incompatible with %s.', mode);
            end
        end
    end
end
