function optimize_gains()
%OPTIMIZE_GAINS Run selected gain-optimization scenarios.
%   Edit opts below to run all scenarios or a selected subset.

clear; close all;
startup;

opts.scenarios = 'all';
opts.duration = 25;
opts.swarmSize = 100;
opts.maxIterations = 100;

opts.functionTolerance = 1e-4;
opts.maxStallIterations = 15;

opts.randomSeed = 20260824;
opts.useParallel = true;

% Search bounds. Gamma bounds are specified in log10 space.
opts.bounds.Kp = [5e-3, 15.0];
opts.bounds.Kd = [1e-3, 10.0];
opts.bounds.lambda = [0.0, 20.0];
opts.bounds.gammaLog10 = [-6.0, -1.0];

fth.opt.GainOptimizer.run(opts);
end
