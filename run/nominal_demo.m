%RUN_NOMINAL_DEMO Nominal hexacopter simulation (no adaptation).
%   Configures a baseline tracking scenario and produces summary plots.

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
simOpts.scriptName    = 'nom_demo';
simOpts.parallelRuns  = true;
cfg.useSimOptions(simOpts);

%% Reference trajectory.

% Analytic Trajectory Options
% trajOpts.name                       = 'lissajous3d';
% trajOpts.goToHoverBeforePathStarts  = false;
% trajOpts.period                     = duration;
% cfg.useTrajectoryOptions(trajOpts);

% Replay Trajectory Options
trajOpts.name                      = 'replay';
trajOpts.replay.id                 = 'ellipse_01_auto'; % Options: 'ellipse_01_auto', 'lemniscate_01_auto', 'RATM_01_auto'
cfg.useTrajectoryOptions(trajOpts);

%% Controller and gains.
ctrlOpts.potential = 'inertia-gain';             % 'log','inertia-gain','body-gain','ref-gain','sym-inv'
ctrlOpts.Kp        = [12.5848, 10.9447, 4.9685, 7.1157, 2.0731, 4.9342]';
ctrlOpts.Kd        = [4.4558, 3.7303, 4.3602, 9.2087, 2.5359, 9.773]';
ctrlOpts.lambda    = [12.2058, 15.0468, 9.165, 5.0389, 8.4363, 6.794]'; % composite-variable coupling

ctrlOpts.paramInit              = 'vehicle-slight-dev';

% Payload-tuned alternative (keep commented while usePayloadOptions is disabled):
% ctrlOpts.Kp        = [14.9141, 14.8619, 15, 15, 9.3736, 15]';
% ctrlOpts.Kd        = [10, 0.001, 7.9213, 0.001, 0.001, 2.0136]';
% ctrlOpts.lambda    = [0, 6.4749, 0, 0, 13.3918, 0]';
% ctrlOpts.paramInit = 'mid-vehicle-payload';

ctrlOpts.coriolisFactorization  = 'consistent';    % 'basic', 'consistent'
cfg.useControllerOptions(ctrlOpts);

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

runOpts.plotMode     = 'summary';   % 'summary', 'all', or 'none'
runOpts.displayPlots = false;
runOpts.saveSimData  = false;
sim.run(runOpts);
