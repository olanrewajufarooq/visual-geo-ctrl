%RUN_NOMINAL_CORIOLIS_COMPARISON Compare basic vs consistent Coriolis factorizations.
%   Runs the same nominal scenario twice via the batch system, once per
%   Coriolis factorization form, and saves full plots and sim data.

%% Clean workspace and load project paths.
clear; close all;
startup;

%% Build a fresh configuration with defaults.
cfg = fth.sim.Config();

%% Scenario duration in seconds.
duration = 25;

%% Simulation timing.
simOpts.dt           = 0.005;
simOpts.duration     = duration;
simOpts.controlDt    = 0.01;
simOpts.adaptationDt = 0.005;
simOpts.enableSafety  = false;
simOpts.runNames     = {'euclid-basic', 'euclid-consistent', 'breg-basic', 'breg-consistent'};
simOpts.scriptName   = 'adapt_coriolis_comp';
simOpts.parallelRuns = true;
cfg.useSimOptions(simOpts);

%% Reference trajectory.

% Analytic Trajectory Options
% trajOpts.name                       = 'lissajous3d';
% trajOpts.goToHoverBeforePathStarts  = false;
% trajOpts.period                     = duration;
% cfg.useTrajectoryOptions(trajOpts);

% Replay Trajectory Options
trajOpts.name      = 'replay';
trajOpts.replay.id                 = 'lemniscate_01_auto'; % Options: 'ellipse_01_auto', 'lemniscate_01_auto', 'RATM_01_auto'
cfg.useTrajectoryOptions(trajOpts);

%% Controller and gains.
ctrlOpts.potential    = 'inertia-gain';             % 'log','inertia-gain','body-gain','ref-gain','sym-inv'
ctrlOpts.Kp           = [ ...
                          12.8028, 8.5605, 13.5362, 7.5227, 4.5456, 10.7101; ... % euclid-basic
                          5.064, 6.3048, 10.1897, 0.8367, 2.329, 5.917; ... % euclid-consistent
                          14.1025, 4.9611, 14.3137, 10.6738, 4.3983, 5.2758; ... % bregman-basic
                          12.2846, 0.5096, 8.4703, 4.2418, 4.0902, 4.3388; ... % bregman-consistent
                        ];
ctrlOpts.Kd           = [ ...
                          5.3874, 5.9667, 3.4602, 9.5722, 9.8436, 8.597; ... % euclid-basic
                          4.1183, 2.8999, 8.369, 8.2987, 7.9422, 5.016; ... % euclid-consistent
                          4.0496, 5.6882, 6.0132, 9.7956, 1.9258, 8.8811; ... % bregman-basic
                          3.5847, 3.068, 5.7808, 7.4322, 9.9078, 8.2202; ... % bregman-consistent
                        ];
ctrlOpts.lambda       = [ ...
                          14.175, 2.3692, 10.0534, 6.4772, 4.1988, 8.1543; ... % euclid-basic
                          9.2286, 8.1794, 10.0842, 9.0037, 12.2946, 19.2356; ... % euclid-consistent
                          7.7375, 2.4013, 12.145, 4.8512, 6.4212, 12.8789; ... % bregman-basic
                          12.8684, 4.9496, 16.6815, 11.6474, 19.3227, 17.6787; ... % bregman-consistent
                        ]; % composite-variable coupling: s = Ve + diag(lambda)*eH

ctrlOpts.paramInit    = 'vehicle-plus-payload';                 % 'vehicle','vehicle-plus-payload','mid-vehicle-payload','vehicle-plus-payload-higher','vehicle-slight-dev','random', or 10x1 custom theta
ctrlOpts.coriolisForm = {'basic', 'consistent', 'basic', 'consistent'};    % cell array triggers one batch run per form
cfg.useControllerOptions(ctrlOpts);

% Adaptation.
adaptOpts.type  = {'euclidean', 'euclidean', 'bregman', 'bregman'};          % 'none','euclidean','bregman'
adaptOpts.Gamma = { ...
                    [0.02781, 0.008174, 0.03803, 0.009728, 9.28e-5, 4.543e-4, 0.002594, 8.244e-6, 3.667e-5, 1.222e-4], ...
                    [0.09001, 0.1, 0.08306, 0.1, 0.002964, 0.09216, 0.1, 3.287e-4, 1e-6, 0.006302], ...
                    0.003593, ...
                    6.417e-6 ...
                  }; % Gamma values
adaptOpts.useBackTracking = true;
cfg.useAdaptationOptions(adaptOpts);

% Payload-tuned alternatives, in the same basic/consistent and
% Euclidean/Bregman order (keep commented while usePayloadOptions is disabled):
% ctrlOpts.Kp = [ ...
%   12.8028, 8.5605, 13.5362, 7.5227, 4.5456, 10.7101; ... % euclid-basic
%    5.064, 6.3048, 10.1897, 0.8367, 2.329, 5.917; ... % euclid-consistent
%   14.1025, 4.9611, 14.3137, 10.6738, 4.3983, 5.2758; ... % bregman-basic
%   12.2846, 0.5096, 8.4703, 4.2418, 4.0902, 4.3388; ... % bregman-consistent
% ];
% ctrlOpts.Kd = [ ...
%    5.3874, 5.9667, 3.4602, 9.5722, 9.8436, 8.597; ... % euclid-basic
%    4.1183, 2.8999, 8.369, 8.2987, 7.9422, 5.016; ... % euclid-consistent
%    4.0496, 5.6882, 6.0132, 9.7956, 1.9258, 8.8811; ... % bregman-basic
%    3.5847, 3.068, 5.7808, 7.4322, 9.9078, 8.2202; ... % bregman-consistent
% ];
% ctrlOpts.lambda = [ ...
%   14.175, 2.3692, 10.0534, 6.4772, 4.1988, 8.1543; ... % euclid-basic
%    9.2286, 8.1794, 10.0842, 9.0037, 12.2946, 19.2356; ... % euclid-consistent
%    7.7375, 2.4013, 12.145, 4.8512, 6.4212, 12.8789; ... % bregman-basic
%   12.8684, 4.9496, 16.6815, 11.6474, 19.3227, 17.6787; ... % bregman-consistent
% ];
% adaptOpts.Gamma = { ...
%   [0.01732, 4.361e-6, 2.78e-6, 1.196e-5, 0.003379, 4.533e-6, 1.078e-4, 6.019e-5, 1.629e-6, 0.02391], ...
%   [0.002023, 0.1, 0.08423, 0.043, 9.723e-6, 1e-6, 5.024e-6, 1.27e-6, 3.725e-6, 1.699e-5], ...
%   0.001, ...
%   0.1 ...
% };
% ctrlOpts.paramInit = 'mid-vehicle-payload';

% Payload schedule (mass drop event).
payloadOpts.mass     = 1.5;
payloadOpts.position = [0.115; 0.05; -0.25];
payloadOpts.dims     = [0.20, 0.20, 0.10];
payloadOpts.dropTime = 2*duration/3;
cfg.usePayloadOptions(payloadOpts);

%% Visualization.
vizOpts.enable      = false;
vizOpts.liveSummary = false;
vizOpts.updateRate  = 500;
vizOpts.embedUrdf   = false;
vizOpts.plotLayout  = 'column-major';
cfg.useVizOptions(vizOpts);

cfg.done();

%% Run the simulation.
sim = fth.sim.SimRunner(cfg);
sim.setup();

runOpts.plotMode      = 'all';   % 'summary', 'all', or 'none'
runOpts.displayPlots  = false;
runOpts.saveSimData   = false;
runOpts.cummPlotModes = {'spd', 'tracking_rmse', 'estimation_nrmse'};
sim.run(runOpts);
