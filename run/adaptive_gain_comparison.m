%RUN_ADAPTIVE_GAIN_COMPARISON Batch gain comparison: Euclidean and Bregman adaptation.
%   Runs the Euclidean and Bregman gain configurations on one replay
%   trajectory in a single batch.

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
simOpts.runNames     = {'baseline', ...
                        'euclid-base-gain', 'euclid-high-inertia', 'euclid-high-cog', ...
                        'breg-low', 'breg-mid', 'breg-high' ...
                        };
simOpts.scriptName   = 'adapt_gain_comp';
simOpts.parallelRuns = true;
cfg.useSimOptions(simOpts);

%% Reference trajectory.

% Analytic Trajectory Options
% trajOpts.names                     = {'circle', 'lissajous3d', 'helix3d', 'poly3d'};
% trajOpts.goToHoverBeforePathStarts = false;
% trajOpts.period                    = [duration, duration, duration/2, duration/2];
% cfg.useTrajectoryOptions(trajOpts);

% Replay Trajectory Options
trajOpts.name      = 'replay';
trajOpts.replay.id                 = 'lemniscate_01_auto'; % Options: 'ellipse_01_auto', 'lemniscate_01_auto', 'RATM_01_auto'
cfg.useTrajectoryOptions(trajOpts);

%% Controller and gains.
ctrlOpts.potential    = 'inertia-gain';
ctrlOpts.Kp           = [ ...
                          12.8028, 8.5605, 13.5362, 7.5227, 4.5456, 10.7101; ... % baseline/euclidean
                          12.8028, 8.5605, 13.5362, 7.5227, 4.5456, 10.7101; ... % euclid-base-gain
                          12.8028, 8.5605, 13.5362, 7.5227, 4.5456, 10.7101; ... % euclid-high-inertia
                          12.8028, 8.5605, 13.5362, 7.5227, 4.5456, 10.7101; ... % euclid-high-cog
                          14.1025, 4.9611, 14.3137, 10.6738, 4.3983, 5.2758; ... % breg-low
                          14.1025, 4.9611, 14.3137, 10.6738, 4.3983, 5.2758; ... % breg-mid
                          14.1025, 4.9611, 14.3137, 10.6738, 4.3983, 5.2758; ... % breg-high
                        ];
ctrlOpts.Kd           = [ ...
                          5.3874, 5.9667, 3.4602, 9.5722, 9.8436, 8.597; ... % baseline/euclidean
                          5.3874, 5.9667, 3.4602, 9.5722, 9.8436, 8.597; ... % euclid-base-gain
                          5.3874, 5.9667, 3.4602, 9.5722, 9.8436, 8.597; ... % euclid-high-inertia
                          5.3874, 5.9667, 3.4602, 9.5722, 9.8436, 8.597; ... % euclid-high-cog
                          4.0496, 5.6882, 6.0132, 9.7956, 1.9258, 8.8811; ... % breg-low
                          4.0496, 5.6882, 6.0132, 9.7956, 1.9258, 8.8811; ... % breg-mid
                          4.0496, 5.6882, 6.0132, 9.7956, 1.9258, 8.8811; ... % breg-high
                        ];
ctrlOpts.lambda       = [ ...
                          14.175, 2.3692, 10.0534, 6.4772, 4.1988, 8.1543; ... % baseline/euclidean
                          14.175, 2.3692, 10.0534, 6.4772, 4.1988, 8.1543; ... % euclid-base-gain
                          14.175, 2.3692, 10.0534, 6.4772, 4.1988, 8.1543; ... % euclid-high-inertia
                          14.175, 2.3692, 10.0534, 6.4772, 4.1988, 8.1543; ... % euclid-high-cog
                          7.7375, 2.4013, 12.145, 4.8512, 6.4212, 12.8789; ... % breg-low
                          7.7375, 2.4013, 12.145, 4.8512, 6.4212, 12.8789; ... % breg-mid
                          7.7375, 2.4013, 12.145, 4.8512, 6.4212, 12.8789; ... % breg-high
                        ];
ctrlOpts.paramInit    = 'vehicle-plus-payload';  % 'vehicle','vehicle-plus-payload','mid-vehicle-payload','vehicle-plus-payload-higher','vehicle-slight-dev','random', or 10x1 custom theta
ctrlOpts.coriolisForm = 'consistent';
cfg.useControllerOptions(ctrlOpts);

% Adaptation — 4 Euclidean runs then 4 Bregman runs.
adaptOpts.type = {'euclidean', ...
                    'euclidean', 'euclidean', 'euclidean', ...
                    'bregman',   'bregman',   'bregman'};
adaptOpts.Gamma = { ...
    zeros(1, 10), ...                                      % baseline (disabled)
    [0.36, 0.12, 0.12, 0.12, 0.08, 0.08, 0.12, 0.004, 0.004, 0.004], ...  % euclid-base-gain
    [0.72, 0.12, 0.12, 0.12, 3.6, 3.6, 3.6, 0.4, 0.4, 0.4], ...          % euclid-high-inertia
    [0.36, 1.2, 1.2, 1.2, 0.08, 0.08, 0.12, 0.004, 0.004, 0.004], ...      % euclid-high-cog
    1/10, ...                                                   % breg-low
    1/20, ...                                                   % breg-mid
    1/30 };                                                     % breg-high
adaptOpts.useBackTracking = true;
cfg.useAdaptationOptions(adaptOpts);

% Payload schedule (mass drop event).
payloadOpts.mass     = 1.5;
payloadOpts.position = [0.115; 0.05; -0.25];
payloadOpts.dims     = [0.20, 0.20, 0.10];
payloadOpts.dropTime = 2*duration/3;
cfg.usePayloadOptions(payloadOpts);

% Visualization.
vizOpts.enable      = false;
vizOpts.liveSummary = false;
vizOpts.updateRate  = 500;
vizOpts.embedUrdf   = false;
vizOpts.plotLayout  = 'column-major';
cfg.useVizOptions(vizOpts);

cfg.done();

% Run the simulation.
sim = fth.sim.SimRunner(cfg);
sim.setup();

runOpts.plotMode      = 'all'; % 'summary', 'all', or 'none'
runOpts.displayPlots  = false;
runOpts.saveSimData   = false;
runOpts.cummPlotModes = {'tracking_rmse', 'estimation_nrmse'};
sim.run(runOpts);
