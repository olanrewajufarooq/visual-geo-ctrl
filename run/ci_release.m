%RUN_ADAPTIVE_DEMO Adaptive control simulation with payload drop.
%   Demonstrates online parameter adaptation and payload change.

% Clean workspace and load project paths.
clear; close all;
startup;

% Build a fresh configuration with defaults.
cfg = fth.sim.Config();

% Scenario duration in seconds.
duration = 30;

% Simulation timing.
simOpts.dt           = 0.005;
simOpts.duration     = duration;
simOpts.controlDt    = 0.01;
simOpts.adaptationDt = 0.005;
simOpts.scriptName   = 'ci_release';
simOpts.parallelRuns = true;
cfg.useSimOptions(simOpts);

% Reference trajectory batch.
trajOpts.names                   = {'circle', 'lissajous3d', 'helix3d', 'poly3d'};
trajOpts.goToHoverBeforePathStarts = false;
trajOpts.period                  = [duration, duration, duration/2, duration/2];
cfg.useTrajectoryOptions(trajOpts);

% Controller and gains.
ctrlOpts.potential  = 'inertia-gain';             % 'log','inertia-gain','body-gain','ref-gain','sym-inv'
ctrlOpts.Kp         = [5.5, 5.5, 5.5, 5.5, 5.5, 5.5]';
ctrlOpts.Kd         = [2.05, 2.05, 2.05, 2.05, 2.05, 2.05]';
ctrlOpts.paramInit  = 'mid-vehicle-payload';      % 'vehicle','vehicle-plus-payload','mid-vehicle-payload','vehicle-plus-payload-higher','vehicle-slight-dev','random', or 10x1 custom theta
cfg.useControllerOptions(ctrlOpts);

% Adaptation (batch Gamma rows — one row per run).
adaptOpts.type  = 'euclidean';          % 'none','euclidean','bregman'
adaptOpts.Gamma = 1e-2 * [ ...   % pi = [m,hx,hy,hz,Ixx,Iyy,Izz,Ixy,Ixz,Iyz]
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0; ...
     36,  12,  12,  12,   8,   8,  12, 0.4, 0.4, 0.4; ...
     72,  12,  12,  12, 360, 360, 360,  40,  40,  40; ...
     36, 120, 120, 120,   8,   8,  12, 0.4, 0.4, 0.4];  % rows: Run 1–4
cfg.useAdaptationOptions(adaptOpts);

% Payload schedule (mass drop event).
payloadOpts.mass     = 1.5;
payloadOpts.CoG      = [0.115; 0.05; -0.05];
payloadOpts.dropTime = 2*duration/3;
cfg.usePayloadOptions(payloadOpts);

cfg.done();

% Run the simulation.
sim = fth.sim.SimRunner(cfg);
sim.setup();

runOpts.plotMode     = 'all';       % 'summary', 'all', or 'none'
runOpts.displayPlots = false;
runOpts.saveSimData  = false;
sim.run(runOpts);
