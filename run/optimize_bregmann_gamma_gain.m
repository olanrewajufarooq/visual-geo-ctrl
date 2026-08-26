%OPTIMIZE_BREGMANN_GAMMA_GAIN Optimize scalar Bregman Gamma.
startup; fth.opt.GammaOptimizationRunner.run('bregman');

% To provide fixed physical gains directly instead of loading the newest
% general-run report, call the runner with a second argument, for example:
% fth.opt.GammaOptimizationRunner.run('bregman', struct('fixedGains', ...
%     struct('Kp', 5.5*ones(6,1), 'Kd', 2.05*ones(6,1), ...
%     'lambda', [0.5;0.5;0.5;0.2;0.2;0.2], 'Gamma', 1e-3)));
