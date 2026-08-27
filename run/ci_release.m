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
cfg.useSimOptions(simOpts);

% Single trajectory for CI (avoids running all four and wasting runner minutes).
trajOpts.name                      = 'lissajous3d';
trajOpts.goToHoverBeforePathStarts = false;
trajOpts.period                    = duration;
cfg.useTrajectoryOptions(trajOpts);

% Controller and gains.
ctrlOpts.potential  = 'inertia-gain';             % 'log','inertia-gain','body-gain','ref-gain','sym-inv'
ctrlOpts.Kp         = [5.5, 5.5, 5.5, 5.5, 5.5, 5.5]';
ctrlOpts.Kd         = [2.05, 2.05, 2.05, 2.05, 2.05, 2.05]';
ctrlOpts.paramInit  = 'mid-vehicle-payload';      % 'vehicle','vehicle-plus-payload','mid-vehicle-payload','vehicle-plus-payload-higher','vehicle-slight-dev','random', or 10x1 custom theta
cfg.useControllerOptions(ctrlOpts);

% Adaptation — single Euclidean run.
adaptOpts.type  = 'euclidean';
adaptOpts.Gamma = 1e-2 * [36, 12, 12, 12, 8, 8, 12, 0.4, 0.4, 0.4];
cfg.useAdaptationOptions(adaptOpts);

% Payload schedule (mass drop event).
payloadOpts.mass     = 1.5;
payloadOpts.position = [0.115; 0.05; -0.25];
payloadOpts.dims     = [0.20, 0.20, 0.10];
payloadOpts.dropTime = 2*duration/3;
cfg.usePayloadOptions(payloadOpts);

% Disable interactive visualization while retaining plot generation below.
vizOpts.enable      = false;
vizOpts.liveSummary = false;
cfg.useVizOptions(vizOpts);

cfg.done();

% Run the simulation.
sim = fth.sim.SimRunner(cfg);
sim.setup();

runOpts.plotMode     = 'all';       % 'summary', 'all', or 'none'
runOpts.displayPlots = false;
runOpts.saveSimData  = false;
sim.run(runOpts);
