%RUN_ADAPTIVE_REPRODUCE Quick reproduction of the paper's adaptive result.
%   Short simulation with Bregman adaptation and a mid-flight payload drop.
%   Intended as the first script a reader runs after cloning the repository.

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
simOpts.scriptName   = 'reproduce';
cfg.useSimOptions(simOpts);

%% Reference trajectory.

% Analytic Trajectory Options
% trajOpts.name                      = 'lissajous3d';
% trajOpts.goToHoverBeforePathStarts = false;
% trajOpts.period                    = duration;
% cfg.useTrajectoryOptions(trajOpts);

% Replay Trajectory Options
trajOpts.name      = 'replay';
trajOpts.replay.id = 'ellipse_01_auto';
cfg.useTrajectoryOptions(trajOpts);

%% Controller and gains.
ctrlOpts.potential = 'inertia-gain';
ctrlOpts.Kp        = [5.5, 5.5, 5.5, 5.5, 5.5, 5.5]';
ctrlOpts.Kd        = [2.05, 2.05, 2.05, 2.05, 2.05, 2.05]';
ctrlOpts.paramInit = 'vehicle';  % 'vehicle','vehicle-plus-payload','mid-vehicle-payload','vehicle-plus-payload-higher','vehicle-slight-dev','random', or 10x1 custom theta
% Use 'vehicle' when payload options are disabled; use a payload-specific mode when a payload is configured.
cfg.useControllerOptions(ctrlOpts);

% Adaptation.
adaptOpts.type  = 'bregman';
adaptOpts.Gamma = 1/10;
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
cfg.useVizOptions(vizOpts);

cfg.done();

% Run the simulation.
sim = fth.sim.SimRunner(cfg);
sim.setup();

runOpts.plotMode     = 'all';
runOpts.displayPlots = false;
runOpts.saveSimData  = false;
sim.run(runOpts);
