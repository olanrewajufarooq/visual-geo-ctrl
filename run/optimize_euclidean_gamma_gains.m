function optimize_euclidean_gamma_gains(overrides)
%OPTIMIZE_EUCLIDEAN_GAMMA_GAINS Optimize only the ten Euclidean Gammas.
%   The fixed Kp, Kd, lambda, and starting Gamma values are loaded from the
%   explicitly supplied general-run report directory.
    startup;
    opts = fth.opt.GainOptimizationUtils.defaults();
    opts.outputRoot = fullfile('results', 'tuning', 'euclidean-gamma');
    opts.cacheRoot = fullfile(opts.outputRoot, 'cache');
    opts.clearCache = false;
    if nargin < 1 || isempty(overrides) || ~isfield(overrides, 'sourceReportDir')
        error('fth:GainOptimizer:MissingSourceReport', ...
            'Provide overrides.sourceReportDir for the completed general-run reports.');
    end
    names = fieldnames(overrides);
    for i = 1:numel(names), opts.(names{i}) = overrides.(names{i}); end

    catalog = fth.opt.GainOptimizationScenario.catalog();
    scenarios = catalog(strcmp({catalog.adaptation}, 'euclidean'));
    if ~(ischar(opts.scenarios) || (isstring(opts.scenarios) && isscalar(opts.scenarios))) || ...
            ~strcmpi(char(opts.scenarios), 'all')
        scenarios = fth.opt.GainOptimizationScenario.select(scenarios, opts.scenarios);
    end
    for i = 1:numel(scenarios)
        scenario = scenarios(i);
        fixed = fth.opt.GainOptimizationIO.loadBestVector(opts.sourceReportDir, ...
            scenario.id, 28, 'euclidean');
        localOpts = opts;
        localOpts.scenarios = scenario.id;
        localOpts.initialVector = fixed;
        localOpts.optimizationMask = false(1, 28);
        localOpts.optimizationMask(19:28) = true;
        fth.opt.GainOptimizer.run(localOpts);
    end
end
