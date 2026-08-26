%OPTIMIZE_EUCLIDEAN_GAMMA_GAINS Optimize the ten Euclidean Gammas.
startup;
opt = struct();

%% To select a specific source directory, define it on opt before the call:

% opt.sourceReportDir = 'results/tuning/20260826_174743';

%% To provide fixed physical gains directly, define opt.fixedGains instead:

% opt.fixedGains.Kp = 5.5*ones(6,1);
% opt.fixedGains.Kd = 2.05*ones(6,1);
% opt.fixedGains.lambda = [0.5;0.5;0.5;0.2;0.2;0.2];
% opt.fixedGains.Gamma = 1e-3*ones(10,1);

%% Running Optimization

fth.opt.GammaOptimizationRunner.run('euclidean', opt);
