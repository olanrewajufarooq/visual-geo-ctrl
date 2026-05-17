%RUN_NOMINAL_CORIOLIS_COMPARISON Compare basic vs consistent Coriolis factorizations.
%   Runs the same nominal scenario twice via the batch system, once per
%   Coriolis factorization form, and saves full plots and sim data.

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
simOpts.runNames     = {'euclid-basic', 'euclid-consistent', 'breg-basic', 'breg-consistent'};
simOpts.scriptName   = 'adapt_coriolis_comp';
simOpts.parallelRuns = true;
cfg.useSimOptions(simOpts);

% Reference trajectory.
trajOpts.name                       = 'lissajous3d';
trajOpts.goToHoverBeforePathStarts  = false;
trajOpts.period                     = duration;
cfg.useTrajectoryOptions(trajOpts);

% Controller and gains.
ctrlOpts.potential    = 'inertia-gain';             % 'log','inertia-gain','body-gain','ref-gain','sym-inv'
ctrlOpts.Kp           = [ ...
                          5.5, 5.5, 5.5, 5.5, 5.5, 5.5; ... % euclid-basic
                          5.5, 5.5, 5.5, 5.5, 5.5, 5.5; ... % euclid-consistent
                          5.5, 5.5, 5.5, 5.5, 5.5, 5.5; ... % bregman-basic
                          5.5, 5.5, 5.5, 5.5, 5.5, 5.5; ... % bregman-consistent
                        ];
ctrlOpts.Kd           = [ ...
                          2.05, 2.05, 2.05, 2.05, 2.05, 2.05; ... % euclid-basic
                          2.05, 2.05, 2.05, 2.05, 2.05, 2.05; ... % euclid-consistent
                          2.05, 2.05, 2.05, 2.05, 2.05, 2.05; ... % bregman-basic
                          2.05, 2.05, 2.05, 2.05, 2.05, 2.05; ... % bregman-consistent
                        ];
ctrlOpts.lambda       = 1e-3 * [ ...
                          5, 5, 5, 50, 50, 50; ...      % euclid-basic
                          5, 5, 5, 50, 50, 50; ...      % euclid-consistent
                          50, 50, 50, 50, 50, 50; ...   % bregman-basic
                          50, 50, 50, 50, 50, 50; ...   % bregman-consistent
                        ]; % composite-variable coupling: s = Ve + diag(lambda)*eH

ctrlOpts.paramInit    = 'mid-vehicle-payload';       % 'vehicle','vehicle-plus-payload','mid-vehicle-payload','vehicle-plus-payload-higher','vehicle-slight-dev','random', or 10x1 custom theta
ctrlOpts.coriolisForm = {'basic', 'consistent', 'basic', 'consistent'};    % cell array triggers one batch run per form
cfg.useControllerOptions(ctrlOpts);

% Adaptation.
adaptOpts.type  = {'euclidean', 'euclidean', 'bregman', 'bregman'};          % 'none','euclidean','bregman'
adaptOpts.Gamma = { ...
                    1e-2 * [36, 12, 12, 12, 8, 8, 12, 0.4, 0.4, 0.4], ...
                    1e-2 * [36, 12, 12, 12, 8, 8, 12, 0.4, 0.4, 0.4], ...
                    1/10, ...
                    1/10 ...
                  }; % Gamma values
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

runOpts.plotMode      = 'all';   % 'summary', 'all', or 'none'
runOpts.displayPlots  = false;
runOpts.saveSimData   = false;
runOpts.cummPlotModes = {'spd', 'tracking_rmse', 'estimation_nrmse'};
sim.run(runOpts);
