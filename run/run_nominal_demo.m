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
trajOpts.replay.id                 = 'ellipse_01_auto';
cfg.useTrajectoryOptions(trajOpts);

%% Controller and gains.
ctrlOpts.potential = 'inertia-gain';             % 'log','inertia-gain','body-gain','ref-gain','sym-inv'
ctrlOpts.Kp        = [5.5, 5.5, 5.5, 5.5, 5.5, 5.5]';
ctrlOpts.Kd        = [4.05, 4.05, 4.05, 2.05, 2.05, 2.05]';
ctrlOpts.lambda    = 1e-1 * [5, 5, 5, 2, 2, 2];        % composite-variable coupling: s = Ve + diag(lambda)*eH

ctrlOpts.paramInit              = 'vehicle';       % 'vehicle','vehicle-plus-payload','mid-vehicle-payload','vehicle-plus-payload-higher','vehicle-slight-dev','random', or 10x1 custom theta
ctrlOpts.coriolisFactorization  = 'consistent';    % 'basic', 'consistent'
cfg.useControllerOptions(ctrlOpts);

%% Visualization.
vizOpts.enable      = true;
vizOpts.liveSummary = true;
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
