%OPTIMIZE_BREGMANN_GAMMA_GAIN Optimize only scalar Bregman Gamma.
%   Edit sourceReportDir and opts below, then run this script from the
%   repository root. Fixed controller gains come from the selected report.

clear; close all;
startup;

sourceReportDir = ''; % e.g. 'results/tuning/20260826_174743'
if isempty(sourceReportDir)
    error('fth:GainOptimizer:MissingSourceReport', ...
        'Set sourceReportDir to the completed general-run report root.');
end

opts = fth.opt.GainOptimizationUtils.defaults();
opts.scenarios = 'all';
opts.duration = 25;
opts.swarmSize = 100;
opts.maxIterations = 100;
opts.functionTolerance = 1e-4;
opts.maxStallIterations = 15;
opts.randomSeed = 20260824;
opts.useParallel = true;
opts.clearCache = false;
opts.outputRoot = fullfile('results', 'tuning', 'bregman-gamma');
opts.cacheRoot = fullfile(opts.outputRoot, 'cache');

catalog = fth.opt.GainOptimizationScenario.catalog();
scenarios = catalog(strcmp({catalog.adaptation}, 'bregman'));
if ~(ischar(opts.scenarios) || (isstring(opts.scenarios) && isscalar(opts.scenarios))) || ...
        ~strcmpi(char(opts.scenarios), 'all')
    scenarios = fth.opt.GainOptimizationScenario.select(scenarios, opts.scenarios);
end
for i = 1:numel(scenarios)
    scenario = scenarios(i);
    fixed = fth.opt.GainOptimizationIO.loadBestVector(sourceReportDir, ...
        scenario.id, 19, 'bregman');
    localOpts = opts;
    localOpts.scenarios = scenario.id;
    localOpts.initialVector = fixed;
    localOpts.optimizationMask = false(1, 19);
    localOpts.optimizationMask(19) = true;
    fth.opt.GainOptimizer.run(localOpts);
end
