function optimize_bregmann_gamma_gain(overrides)
%OPTIMIZE_BREGMANN_GAMMA_GAIN Optimize only scalar Bregman Gamma.
%   The fixed Kp, Kd, lambda, and starting Gamma are loaded from the
%   explicitly supplied general-run report directory.
    startup;
    opts = fth.opt.GainOptimizationUtils.defaults();
    opts.outputRoot = fullfile('results', 'tuning', 'bregman-gamma');
    opts.cacheRoot = fullfile(opts.outputRoot, 'cache');
    opts.clearCache = false;
    if nargin < 1 || isempty(overrides) || ~isfield(overrides, 'sourceReportDir')
        error('fth:GainOptimizer:MissingSourceReport', ...
            'Provide overrides.sourceReportDir for the completed general-run reports.');
    end
    names = fieldnames(overrides);
    for i = 1:numel(names), opts.(names{i}) = overrides.(names{i}); end

    catalog = fth.opt.GainOptimizationScenario.catalog();
    scenarios = catalog(strcmp({catalog.adaptation}, 'bregman'));
    if ~(ischar(opts.scenarios) || (isstring(opts.scenarios) && isscalar(opts.scenarios))) || ...
            ~strcmpi(char(opts.scenarios), 'all')
        scenarios = fth.opt.GainOptimizationScenario.select(scenarios, opts.scenarios);
    end
    for i = 1:numel(scenarios)
        scenario = scenarios(i);
        fixed = fth.opt.GainOptimizationIO.loadBestVector(opts.sourceReportDir, ...
            scenario.id, 19, 'bregman');
        localOpts = opts;
        localOpts.scenarios = scenario.id;
        localOpts.initialVector = fixed;
        localOpts.optimizationMask = false(1, 19);
        localOpts.optimizationMask(19) = true;
        fth.opt.GainOptimizer.run(localOpts);
    end
end
