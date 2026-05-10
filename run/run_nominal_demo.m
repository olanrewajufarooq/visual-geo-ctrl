%RUN_NOMINAL_DEMO Nominal hexacopter simulation (no adaptation).
%   Configures a baseline tracking scenario and produces summary plots.

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
trajOpts.name = 'lissajous3d';
trajOpts.goToHoverBeforePathStarts = false;
trajOpts.period                  = duration;
cfg.useTrajectoryOptions(trajOpts);

% Controller and gains.
ctrlOpts.potential = 'inertia-gain';             % 'log','inertia-gain','body-gain','ref-gain','sym-inv'
ctrlOpts.Kp        = [5.5, 5.5, 5.5, 5.5, 5.5, 5.5]';
ctrlOpts.Kd        = [2.05, 2.05, 2.05, 2.05, 2.05, 2.05]';
ctrlOpts.lambda    = [0, 0, 0, 1, 1, 1];        % composite-variable coupling: s = Ve + diag(lambda)*eH
ctrlOpts.coriolisFactorization = 'consistent';    % 'basic', 'consistent'
cfg.useControllerOptions(ctrlOpts);

% Visualization.
vizOpts.enable      = true;
vizOpts.liveSummary = true;
vizOpts.updateRate  = 500;
vizOpts.embedUrdf   = false;
vizOpts.plotLayout  = 'column-major';
cfg.useVizOptions(vizOpts);

cfg.done();

% Run the simulation.
sim = fth.sim.SimRunner(cfg);
sim.setup();

runOpts.plotMode     = 'summary';   % 'summary', 'all', or 'none'
runOpts.displayPlots = false;
runOpts.saveSimData  = false;
sim.run(runOpts);
