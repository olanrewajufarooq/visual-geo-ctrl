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
ctrlOpts.Kp         = [12.8028, 8.5605, 13.5362, 7.5227, 4.5456, 10.7101]';
ctrlOpts.Kd         = [5.3874, 5.9667, 3.4602, 9.5722, 9.8436, 8.597]';
ctrlOpts.paramInit  = 'vehicle-plus-payload';    % 'vehicle','vehicle-plus-payload','mid-vehicle-payload','vehicle-plus-payload-higher','vehicle-slight-dev','random', or 10x1 custom theta
cfg.useControllerOptions(ctrlOpts);

% Adaptation — single Euclidean run.
adaptOpts.type  = 'euclidean';
adaptOpts.Gamma = [0.01732, 4.361e-6, 2.78e-6, 1.196e-5, 0.003379, 4.533e-6, 1.078e-4, 6.019e-5, 1.629e-6, 0.02391];
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
