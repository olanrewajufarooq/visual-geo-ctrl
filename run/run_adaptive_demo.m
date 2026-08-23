%RUN_ADAPTIVE_DEMO Adaptive control simulation with payload drop.
%   Demonstrates online parameter adaptation and payload change.

%% Clean workspace and load project paths.
clear; close all;
startup;

%% Build a fresh configuration with defaults.
cfg = fth.sim.Config();

%% Scenario duration in seconds.
duration = 60;

%% Simulation timing.
simOpts.dt           = 0.005;
simOpts.duration     = duration;
simOpts.controlDt    = 0.01;
simOpts.adaptationDt = 0.005;
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
trajOpts.replay.id = 'ellipse_01_auto';
cfg.useTrajectoryOptions(trajOpts);

%% Controller and gains.
ctrlOpts.potential    = 'inertia-gain';             % 'log','inertia-gain','body-gain','ref-gain','sym-inv'
ctrlOpts.Kp           = [5.5, 5.5, 5.5, 5.5, 5.5, 5.5]';
ctrlOpts.Kd           = [2.05, 2.05, 2.05, 2.05, 2.05, 2.05]';
ctrlOpts.lambda       = 1e-1 * [5, 5, 5, 2, 2, 2]; % composite-variable coupling: s = Ve + diag(lambda)*eH

ctrlOpts.paramInit    = 'vehicle';                  % 'vehicle','vehicle-plus-payload','mid-vehicle-payload','vehicle-plus-payload-higher','vehicle-slight-dev','random', or 10x1 custom theta
% Use 'vehicle' when payload options are disabled; use a payload-specific mode when a payload is configured.
ctrlOpts.coriolisForm = 'basic';    % 'basic', 'consistent'
cfg.useControllerOptions(ctrlOpts);

% Adaptation.
adaptOpts.type  = 'bregman';          % 'none','euclidean','bregman'
% adaptOpts.Gamma = 1e-2 * [36, 12, 12, 12, 8, 8, 12, 0.4, 0.4, 0.4];  % Gamma for Euclidean. Corr. to pi = [m,hx,hy,hz,Ixx,Iyy,Izz,Ixy,Ixz,Iyz]
adaptOpts.Gamma = 1/10; % Gamma value for Bregman
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
