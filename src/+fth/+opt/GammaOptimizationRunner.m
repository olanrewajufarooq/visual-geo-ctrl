classdef GammaOptimizationRunner
    %GAMMAOPTIMIZATIONRUNNER Run the configured Gamma-only workflow.
    methods (Static)
        function run(mode, overrides)
            %RUN Optimize Gamma while holding controller gains fixed.
            arguments
                mode (1,:) char
                overrides struct = struct()
            end
            mode = lower(mode);
            if ~ismember(mode, {'bregman', 'euclidean'})
                error('fth:GammaOptimizationRunner:InvalidMode', ...
                    'Mode must be bregman or euclidean.');
            end
            opts = fth.opt.GainOptimizationUtils.defaults();
            opts.scenarios = 'all';
            opts.outputRoot = fullfile('results', 'tuning', [mode '-gamma']);
            opts.cacheRoot = fullfile(opts.outputRoot, 'cache');
            opts.clearCache = false;
            sourceReportDir = '';
            fixedGains = [];
            paramInit = '';
            replayId = '';
            names = fieldnames(overrides);
            for i = 1:numel(names)
                if strcmp(names{i}, 'bounds') && isstruct(overrides.bounds)
                    boundNames = fieldnames(overrides.bounds);
                    for j = 1:numel(boundNames)
                        opts.bounds.(boundNames{j}) = overrides.bounds.(boundNames{j});
                    end
                elseif isfield(opts, names{i})
                    opts.(names{i}) = overrides.(names{i});
                end
            end
            if isfield(overrides, 'sourceReportDir'), sourceReportDir = overrides.sourceReportDir; end
            if isfield(overrides, 'fixedGains'), fixedGains = overrides.fixedGains; end
            if isfield(overrides, 'paramInit'), paramInit = char(string(overrides.paramInit)); end
            if isfield(overrides, 'replayId'), replayId = char(string(overrides.replayId)); end

            catalog = fth.opt.GainOptimizationScenario.catalog();
            scenarios = catalog(strcmp({catalog.adaptation}, mode));
            if ~(ischar(opts.scenarios) || (isstring(opts.scenarios) && isscalar(opts.scenarios))) || ...
                    ~strcmpi(char(opts.scenarios), 'all')
                scenarios = fth.opt.GainOptimizationScenario.select(scenarios, opts.scenarios);
            end
            dimension = 18 + fth.opt.GainOptimizationUtils.gammaWidth(mode);
            mask = false(1, dimension); mask(19:dimension) = true;
            for i = 1:numel(scenarios)
                scenario = scenarios(i);
                if isempty(fixedGains)
                    fixed = fth.opt.GainOptimizationSource.loadVector(sourceReportDir, ...
                        fullfile('results', 'tuning'), scenario.id, dimension, mode);
                else
                    fixed = fth.opt.GainOptimizationSource.gainsToVector(fixedGains, mode);
                end
                localOpts = opts;
                localOpts.scenarios = scenario.id;
                if ~isempty(paramInit), localOpts.paramInit = paramInit; end
                if ~isempty(replayId), localOpts.replayId = replayId; end
                localOpts.initialVector = fixed;
                localOpts.optimizationMask = mask;
                fth.opt.GainOptimizer.run(localOpts);
            end
        end
    end
end
