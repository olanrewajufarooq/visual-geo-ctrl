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
cfg.useSimOptions(simOpts);

% Reference trajectory.
trajOpts.name                    = 'infinity3dmod';
trajOpts.cycles                  = 2.25;
trajOpts.goToHoverBeforePathStarts = true;
trajOpts.method                  = 'precomputed';   % 'precomputed','modelreference'
cfg.useTrajectoryOptions(trajOpts);

% Controller and gains.
ctrlOpts.potential = 'log';             % 'log','inertia-gain','body-gain','ref-gain','sym-inv'
ctrlOpts.Kp        = [5.5, 5.5, 5.5, 5.5, 5.5, 5.5]';
ctrlOpts.Kd        = [2.05, 2.05, 2.05, 2.05, 2.05, 2.05]';
cfg.useControllerOptions(ctrlOpts);

% Adaptation.
adaptOpts.type  = 'euclidean';          % 'none','euclidean','geo-aware'
adaptOpts.Gamma = 1e-2 * [8, 8, 12, 0.4, 0.4, 0.4, 36, 12, 12, 12];
adaptOpts.init  = 'random';
cfg.useAdaptationOptions(adaptOpts);

% Payload schedule (mass drop event).
payloadOpts.mass     = 1.5;
payloadOpts.CoG      = [0.115; 0.05; -0.05];
payloadOpts.dropTime = 2*duration/3;
cfg.usePayloadOptions(payloadOpts);

% Visualization.
vizOpts.enable      = true;
vizOpts.liveSummary = true;
vizOpts.updateRate  = 500;
vizOpts.embedUrdf   = true;
vizOpts.plotLayout  = 'column-major';
cfg.useVizOptions(vizOpts);

cfg.done();

% Run the simulation, save plots silently, and skip sim_data.mat.
sim = fth.sim.SimRunner(cfg);
sim.setup();
sim.run( ...
   'summary', ...       % plotting mode: 'summary', 'all', or 'none'
   false, ...           % display plots while saving
   false ...            % save sim_data.mat
);
