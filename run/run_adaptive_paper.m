%RUN_ADAPTIVE_PAPER Batch adaptive simulation for paper results.
%   Runs multiple trajectories and gain configurations.

% Clean workspace and load project paths.
clear; close all;
startup;

% Build a fresh configuration with defaults.
cfg = fth.sim.Config();

% Scenario duration in seconds.
duration = 60;

% Simulation timing.
simOpts.dt           = 0.005;
simOpts.duration     = duration;
simOpts.controlDt    = 0.01;
simOpts.adaptationDt = 0.005;
cfg.useSimOptions(simOpts);

% Reference trajectory batch.
trajOpts.names                   = {'circle', 'infinity3dmod', 'lissajous3d', 'helix3d', 'poly3d'};
trajOpts.cycles                  = 1.25;
trajOpts.goToHoverBeforePathStarts = true;
cfg.useTrajectoryOptions(trajOpts);

% Controller and gains.
ctrlOpts.potential = 'log';             % 'log','inertia-gain','body-gain','ref-gain','sym-inv'
ctrlOpts.Kp        = [5.5, 5.5, 5.5, 5.5, 5.5, 5.5]';
ctrlOpts.Kd        = [2.05, 2.05, 2.05, 2.05, 2.05, 2.05]';
cfg.useControllerOptions(ctrlOpts);

% Adaptation (batch Gamma rows — one row per run).
adaptOpts.type  = 'euclidean';          % 'none','euclidean','geo-aware'
adaptOpts.Gamma = 1e-2 * [ ...
      0,   0,   0,   0,   0,   0,  0,   0,   0,   0; ...
      8,   8,  12, 0.4, 0.4, 0.4, 36,  12,  12,  12; ...
    360, 360, 360,  40,  40,  40, 72,  12,  12,  12; ...
      8,   8,  12, 0.4, 0.4, 0.4, 36, 120, 120, 120];  % rows: Run 1–4
adaptOpts.init  = 'nominal';
cfg.useAdaptationOptions(adaptOpts);

% Payload schedule (mass drop event).
payloadOpts.mass     = 1.5;
payloadOpts.CoG      = [0.115; 0.05; -0.05];
payloadOpts.dropTime = 2*duration/3;
cfg.usePayloadOptions(payloadOpts);

cfg.done();

% Run the simulation, save plots silently, and skip sim_data.mat.
sim = fth.sim.SimRunner(cfg);
sim.setup();
sim.run( ...
   'all', ...           % plotting mode: 'summary', 'all', or 'none'
   false, ...           % display plots while saving
   false ...            % save sim_data.mat
);
