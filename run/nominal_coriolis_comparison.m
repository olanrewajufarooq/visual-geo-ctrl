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
simOpts.dt            = 0.005;
simOpts.duration      = duration;
simOpts.controlDt     = 0.005;
simOpts.enableSafety   = false;
simOpts.runNames      = {'basic', 'consistent'};
simOpts.scriptName    = 'nom_coriolis_comp';
simOpts.parallelRuns  = true;
cfg.useSimOptions(simOpts);

%% Reference trajectory.

% Analytic Trajectory Options
% trajOpts.name                      = 'lissajous3d';
% trajOpts.goToHoverBeforePathStarts = false;
% trajOpts.period                    = duration;
% cfg.useTrajectoryOptions(trajOpts);

% Replay Trajectory Options
trajOpts.name                      = 'replay';
trajOpts.replay.id                 = 'ellipse_01_auto'; % Options: 'ellipse_01_auto', 'lemniscate_01_auto', 'RATM_01_auto'
cfg.useTrajectoryOptions(trajOpts);

%% Controller and gains.
ctrlOpts.potential    = 'inertia-gain';             % 'log','inertia-gain','body-gain','ref-gain','sym-inv'
ctrlOpts.coriolisForm = {'basic', 'consistent'};    % cell array triggers one batch run per form
ctrlOpts.Kp           = [ ...
                          2.268, 1.2305, 2.91, 3.3161, 2.94, 2.5273; ... % basic
                          12.5848, 10.9447, 4.9685, 7.1157, 2.0731, 4.9342; ... % consistent
                        ];
ctrlOpts.Kd           = [ ...
                          2.3213, 2.9351, 6.2079, 5.6503, 3.3681, 9.652; ... % basic
                          4.4558, 3.7303, 4.3602, 9.2087, 2.5359, 9.7730; ... % consistent
                        ];
ctrlOpts.lambda       = [ ...
                          3.7561, 9.7063, 17.4548, 11.1242, 9.8628, 5.8183; ... % basic
                          12.2058, 15.0468, 9.1650, 5.0389, 8.4363, 6.7940; ... % consistent
                        ]; % composite-variable coupling
ctrlOpts.paramInit    = 'vehicle-slight-dev';       % 'vehicle','vehicle-plus-payload','mid-vehicle-payload','vehicle-plus-payload-higher','vehicle-slight-dev','random', or 10x1 custom theta

% Payload-tuned alternatives, in basic/consistent order (keep commented while usePayloadOptions is disabled):
% ctrlOpts.Kp = [ ...
%                  0.005, 15, 2.3207, 0.005, 9.879, 5.4567; ...
%                  14.9141, 14.8619, 15, 15, 9.3736, 15; ...
%                ];
% ctrlOpts.Kd = [ ...
%                  0.001, 6.9414, 10, 1.6363, 0.001, 10; ...
%                  10, 0.001, 7.9213, 0.001, 0.001, 2.0136; ...
%                ];
% ctrlOpts.lambda = [ ...
%                  0, 14.166, 0, 18.9191, 16.444, 4.9797; ...
%                  0, 6.4749, 0, 0, 13.3918, 0; ...
%                ];
% ctrlOpts.paramInit = 'mid-vehicle-payload';
cfg.useControllerOptions(ctrlOpts);

%% Visualization.
vizOpts.enable      = false;
vizOpts.liveSummary = false;
vizOpts.updateRate  = 500;
vizOpts.embedUrdf   = false;
vizOpts.plotLayout  = 'column-major';
cfg.useVizOptions(vizOpts);

cfg.done();

%% Run the simulation — batch system produces one subdirectory per form.
sim = fth.sim.SimRunner(cfg);
sim.setup();

runOpts.plotMode      = 'all';        % 'summary', 'all', or 'none'
runOpts.displayPlots  = false;
runOpts.saveSimData   = false;
runOpts.cummPlotModes = {'tracking_rmse'};
sim.run(runOpts);
