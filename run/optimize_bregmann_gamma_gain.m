%OPTIMIZE_BREGMANN_GAMMA_GAIN Optimize scalar Bregman Gamma.
startup;
opt = struct();
opt.scenarios = {'adaptive-basic-bregman-payload-drop', ...
                 'adaptive-consistent-bregman-payload-drop'};
opt.paramInit = 'vehicle-plus-payload';
opt.replayId = 'lemniscate_01_auto';


%% To select a specific source directory, define it on opt before the call:

opt.sourceReportDir = 'results/tuning/20260826_205119';

%% To provide fixed physical gains directly, define opt.fixedGains instead:

% opt.fixedGains.Kp = 5.5*ones(6,1);
% opt.fixedGains.Kd = 2.05*ones(6,1);
% opt.fixedGains.lambda = [0.5;0.5;0.5;0.2;0.2;0.2];
% opt.fixedGains.Gamma = 1e-3;

% Optional search-bound overrides. Gamma bounds are log10 values.
% opt.bounds = struct();
% opt.bounds.gammaLog10 = [-6, -1];

%% Running Optimization

fth.opt.GammaOptimizationRunner.run('bregman', opt);
