% OPTIMIZE_GAINS Tune scalar transverse gains through the shared objective.
% Edit these settings, then press Run.
replayId = 'lemniscate_01_auto';
duration = 5;
startup;
base = agc.sim.defaultScenario(replayId, 'bregman', 'c2', duration);
decode = @agc.opt.applyTransverseGains;
weights = struct('position', 1, 'attitude', 1, 'effort', 1e-3, 'failure', 1e6);
result = agc.opt.optimize({base}, decode, [0.51, 1e-3, 0.1], [10, 10, 0.9], ...
    struct('weights', weights, 'swarmSize', 20, 'maxIterations', 20));
disp(result);
