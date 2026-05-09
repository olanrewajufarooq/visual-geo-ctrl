%RUN_NOMINAL_DEMO Nominal hexacopter simulation (no adaptation).
%   Configures a baseline tracking scenario and produces summary plots.

% Clean workspace and load project paths.
clear; close all;
startup;

% Build a fresh configuration with defaults.
cfg = fth.sim.Config();

% Simulation timing.
simOpts.dt        = 0.005;
simOpts.duration  = 60;
simOpts.controlDt = 0.005;
cfg.useSimOptions(simOpts);

% Reference trajectory.
trajOpts.name = 'lissajous3d';
cfg.useTrajectoryOptions(trajOpts);

% Controller and gains.
ctrlOpts.potential = 'inertia-gain';    % 'log','inertia-gain','body-gain','ref-gain','sym-inv'
ctrlOpts.Kp        = [5.5, 5.5, 5.5, 5.5, 5.5, 5.5]';
ctrlOpts.Kd        = [2.05, 2.05, 2.05, 2.05, 2.05, 2.05]';
cfg.useControllerOptions(ctrlOpts);

% Visualization.
vizOpts.enable      = true;
vizOpts.liveSummary = true;
vizOpts.updateRate  = 500;
vizOpts.embedUrdf   = false;
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
