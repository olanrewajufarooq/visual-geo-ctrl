%RUN_ADAPTIVE_DEMO Adaptive control simulation with payload drop.
%   Demonstrates online parameter adaptation and payload change.

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
simOpts.scriptName   = 'adapt_demo';
simOpts.parallelRuns = true;
cfg.useSimOptions(simOpts);

%% Reference trajectory.

% Analytic Trajectory Options
% trajOpts.name                       = 'lissajous3d';
% trajOpts.goToHoverBeforePathStarts  = false;
% trajOpts.period                     = duration;
% cfg.useTrajectoryOptions(trajOpts);

% Replay Trajectory Options
trajOpts.name      = 'replay';
trajOpts.replay.id                 = 'ellipse_01_auto'; % Options: 'ellipse_01_auto', 'lemniscate_01_auto', 'RATM_01_auto'
cfg.useTrajectoryOptions(trajOpts);

%% Controller and gains.
ctrlOpts.potential    = 'inertia-gain';             % 'log','inertia-gain','body-gain','ref-gain','sym-inv'
ctrlOpts.Kp           = [6.6208, 0.2757, 0.7529, 4.9531, 1.4358, 0.7464]';
ctrlOpts.Kd           = [2.941, 1.983, 4.2557, 6.3416, 0.0433, 10]';
ctrlOpts.lambda       = [18.7537, 5.8721, 19.9955, 17.2356, 14.5522, 15.0557]'; % composite-variable coupling

ctrlOpts.paramInit    = 'vehicle-slight-dev';                  % 'vehicle','vehicle-plus-payload','mid-vehicle-payload','vehicle-plus-payload-higher','vehicle-slight-dev','random', or 10x1 custom theta
% Use 'vehicle' or 'vehicle-slight-dev' when payload options are disabled; use a payload-specific mode when a payload is configured.
ctrlOpts.coriolisForm = 'basic';    % 'basic', 'consistent'
cfg.useControllerOptions(ctrlOpts);

% Adaptation.
adaptOpts.type  = 'bregman';          % 'none','euclidean','bregman'
% adaptOpts.Gamma = 1e-2 * [36, 12, 12, 12, 8, 8, 12, 0.4, 0.4, 0.4];  % Gamma for Euclidean. Corr. to pi = [m,hx,hy,hz,Ixx,Iyy,Izz,Ixy,Ixz,Iyz]
% Replay accelerations have substantially larger regressor scale than the
% analytic demo; use a conservative Bregman rate for this physical dataset.
adaptOpts.Gamma = 0.0036; % Gamma value for Bregman

% Payload-tuned alternative (keep commented while usePayloadOptions is disabled):
% ctrlOpts.Kp        = [14.1025, 4.9611, 14.3137, 10.6738, 4.3983, 5.2758]';
% ctrlOpts.Kd        = [4.0496, 5.6882, 6.0132, 9.7956, 1.9258, 8.8811]';
% ctrlOpts.lambda    = [7.7375, 2.4013, 12.1450, 4.8512, 6.4212, 12.8789]';
% ctrlOpts.paramInit = 'mid-vehicle-payload';
% adaptOpts.Gamma    = 0.0009;
adaptOpts.useBackTracking = true;
cfg.useAdaptationOptions(adaptOpts);

% Payload schedule (mass drop event).
% payloadOpts.mass     = 1.5;
% payloadOpts.position = [0.115; 0.05; -0.25];
% payloadOpts.dims     = [0.20, 0.20, 0.10];
% payloadOpts.dropTime = 2*duration/3;
% cfg.usePayloadOptions(payloadOpts);

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
