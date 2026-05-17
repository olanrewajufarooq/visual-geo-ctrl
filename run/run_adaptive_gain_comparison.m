%RUN_ADAPTIVE_GAIN_COMPARISON Batch gain comparison: Euclidean and Bregman adaptation.
%   Runs 4 Euclidean gain configs followed by 4 Bregman gain configs
%   across 4 trajectories in a single batch.

% Clean workspace and load project paths.
clear; close all;
startup;

% Build a fresh configuration with defaults.
cfg = fth.sim.Config();

% Scenario duration in seconds.
duration = 60;

% Simulation timing.
simOpts.dt           = 0.005;
simOpts.duration     = duration;
simOpts.controlDt    = 0.01;
simOpts.adaptationDt = 0.005;
simOpts.runNames     = {'baseline', ...
                        'euclid-base-gain', 'euclid-high-inertia', 'euclid-high-cog', ...
                        'breg-low', 'breg-mid', 'breg-high' ...
                        };
simOpts.scriptName   = 'adapt_gain_comp';
simOpts.parallelRuns = true;
cfg.useSimOptions(simOpts);

% Reference trajectory batch.
trajOpts.names                     = {'circle', 'lissajous3d', 'helix3d', 'poly3d'};
trajOpts.goToHoverBeforePathStarts = false;
trajOpts.period                    = [duration, duration, duration/2, duration/2];
cfg.useTrajectoryOptions(trajOpts);

% Controller and gains.
ctrlOpts.potential    = 'inertia-gain';
ctrlOpts.Kp           = [5.5, 5.5, 5.5, 5.5, 5.5, 5.5]';
ctrlOpts.Kd           = [2.05, 2.05, 2.05, 2.05, 2.05, 2.05]';
ctrlOpts.lambda       = 1e-3 * [5, 5, 5, 50, 50, 50];
ctrlOpts.paramInit    = 'mid-vehicle-payload';
ctrlOpts.coriolisForm = 'consistent';
cfg.useControllerOptions(ctrlOpts);

% Adaptation — 4 Euclidean runs then 4 Bregman runs.
adaptOpts.type = {'euclidean', ...
                    'euclidean', 'euclidean', 'euclidean', ...
                    'bregman',   'bregman',   'bregman'};
adaptOpts.Gamma = { ...
    1e-2 * zeros(1, 10), ...                                    % baseline (disabled)
    1e-2 * [36, 12, 12, 12,   8,   8,  12, 0.4, 0.4, 0.4], ...  % euclid-base-gain
    1e-2 * [72, 12, 12, 12, 360, 360, 360,  40,  40,  40], ...  % euclid-high-inertia
    1e-2 * [36,120,120,120,   8,   8,  12, 0.4, 0.4, 0.4], ...  % euclid-high-cog
    1/10, ...                                                   % breg-low
    1/20, ...                                                   % breg-mid
    1/30 };                                                     % breg-high
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

runOpts.plotMode      = 'all';
runOpts.displayPlots  = false;
runOpts.saveSimData   = false;
runOpts.cummPlotModes = {'tracking_rmse', 'estimation_nrmse'};
sim.run(runOpts);
