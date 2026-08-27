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
trajOpts.replay.id                 = 'ellipse_01_auto'; % Options: 'ellipse_01_auto', 'lemniscate_01_auto', 'RATM_01_auto'
cfg.useTrajectoryOptions(trajOpts);

%% Controller and gains.
ctrlOpts.potential    = 'inertia-gain';
ctrlOpts.Kp           = [ ...
                          2.268, 1.2305, 2.91, 3.3161, 2.94, 2.5273; ... % baseline
                          0.236, 1.8053, 0.2848, 0.8799, 0.5816, 0.5564; ... % euclid-base-gain
                          0.236, 1.8053, 0.2848, 0.8799, 0.5816, 0.5564; ... % euclid-high-inertia
                          0.236, 1.8053, 0.2848, 0.8799, 0.5816, 0.5564; ... % euclid-high-cog
                          6.6208, 0.2757, 0.7529, 4.9531, 1.4358, 0.7464; ... % breg-low
                          6.6208, 0.2757, 0.7529, 4.9531, 1.4358, 0.7464; ... % breg-mid
                          6.6208, 0.2757, 0.7529, 4.9531, 1.4358, 0.7464; ... % breg-high
                        ];
ctrlOpts.Kd           = [ ...
                          2.3213, 2.9351, 6.2079, 5.6503, 3.3681, 9.652; ... % baseline
                          0.4658, 1.7862, 9.9645, 9.5947, 9.7699, 8.4356; ... % euclid-base-gain
                          0.4658, 1.7862, 9.9645, 9.5947, 9.7699, 8.4356; ... % euclid-high-inertia
                          0.4658, 1.7862, 9.9645, 9.5947, 9.7699, 8.4356; ... % euclid-high-cog
                          2.941, 1.983, 4.2557, 6.3416, 0.0433, 10; ... % breg-low
                          2.941, 1.983, 4.2557, 6.3416, 0.0433, 10; ... % breg-mid
                          2.941, 1.983, 4.2557, 6.3416, 0.0433, 10; ... % breg-high
                        ];
ctrlOpts.lambda       = [ ...
                          3.7561, 9.7063, 17.4548, 11.1242, 9.8628, 5.8183; ... % baseline
                          6.6353, 5.4944, 18.9476, 20, 12.4559, 18.9255; ... % euclid-base-gain
                          6.6353, 5.4944, 18.9476, 20, 12.4559, 18.9255; ... % euclid-high-inertia
                          6.6353, 5.4944, 18.9476, 20, 12.4559, 18.9255; ... % euclid-high-cog
                          18.7537, 5.8721, 19.9955, 17.2356, 14.5522, 15.0557; ... % breg-low
                          18.7537, 5.8721, 19.9955, 17.2356, 14.5522, 15.0557; ... % breg-mid
                          18.7537, 5.8721, 19.9955, 17.2356, 14.5522, 15.0557; ... % breg-high
                        ];
ctrlOpts.paramInit    = 'vehicle-slight-dev';  % 'vehicle','vehicle-plus-payload','mid-vehicle-payload','vehicle-plus-payload-higher','vehicle-slight-dev','random', or 10x1 custom theta
% Use 'vehicle' or 'vehicle-slight-dev' when payload options are disabled; use a payload-specific mode when a payload is configured.
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
% payloadOpts.mass     = 1.5;
% payloadOpts.position = [0.115; 0.05; -0.25];
% payloadOpts.dims     = [0.20, 0.20, 0.10];
% payloadOpts.dropTime = 2*duration/3;
% cfg.usePayloadOptions(payloadOpts);

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
