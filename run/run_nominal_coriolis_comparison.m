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
simOpts.dt        = 0.005;
simOpts.duration  = duration;
simOpts.controlDt = 0.005;
cfg.useSimOptions(simOpts);

% Reference trajectory.
trajOpts.name                      = 'lissajous3d';
trajOpts.goToHoverBeforePathStarts = false;
trajOpts.period                    = duration;
cfg.useTrajectoryOptions(trajOpts);

% Controller and gains.
ctrlOpts.potential    = 'inertia-gain';             % 'log','inertia-gain','body-gain','ref-gain','sym-inv'
ctrlOpts.coriolisForm = {'basic', 'consistent'};    % cell array triggers one batch run per form
ctrlOpts.Kp           = [5.5, 5.5, 5.5, 5.5, 5.5, 5.5]';
ctrlOpts.Kd           = [4.05, 4.05, 4.05, 2.05, 2.05, 2.05]';
ctrlOpts.lambda       = 1e-1 * [5, 5, 5, 2, 2, 2];        % composite-variable coupling: s = Ve + diag(lambda)*eH
ctrlOpts.paramInit    = 'vehicle-slight-dev';       % 'vehicle','vehicle-plus-payload','mid-vehicle-payload','vehicle-plus-payload-higher','vehicle-slight-dev','random', or 10x1 custom theta
cfg.useControllerOptions(ctrlOpts);

% Visualization.
vizOpts.enable      = true;
vizOpts.liveSummary = true;
vizOpts.updateRate  = 500;
vizOpts.embedUrdf   = false;
vizOpts.plotLayout  = 'column-major';
cfg.useVizOptions(vizOpts);

cfg.done();

% Run the simulation — batch system produces one subdirectory per form.
sim = fth.sim.SimRunner(cfg);
sim.setup();

runOpts.plotMode     = 'all';        % 'summary', 'all', or 'none'
runOpts.displayPlots = false;
runOpts.saveSimData  = false;
sim.run(runOpts);
