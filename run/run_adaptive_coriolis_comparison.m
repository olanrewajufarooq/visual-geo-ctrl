%RUN_NOMINAL_CORIOLIS_COMPARISON Compare basic vs consistent Coriolis factorizations.
%   Runs the same nominal scenario twice via the batch system, once per
%   Coriolis factorization form, and saves full plots and sim data.

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

% Reference trajectory.
trajOpts.name                       = 'lissajous3d';
trajOpts.goToHoverBeforePathStarts  = false;
trajOpts.period                     = duration;
cfg.useTrajectoryOptions(trajOpts);

% Controller and gains.
ctrlOpts.potential    = 'inertia-gain';             % 'log','inertia-gain','body-gain','ref-gain','sym-inv'
ctrlOpts.Kp           = [5.5, 5.5, 5.5, 5.5, 5.5, 5.5]';
ctrlOpts.Kd           = [2.05, 2.05, 2.05, 2.05, 2.05, 2.05]';
ctrlOpts.lambda       = 1e-3 * [5, 5, 5, 50, 50, 50]; % composite-variable coupling: s = Ve + diag(lambda)*eH

ctrlOpts.paramInit    = 'mid-vehicle-payload';       % 'vehicle','vehicle-plus-payload','mid-vehicle-payload','vehicle-plus-payload-higher','vehicle-slight-dev','random', or 10x1 custom theta
ctrlOpts.coriolisForm = {'basic', 'consistent'};    % cell array triggers one batch run per form
cfg.useControllerOptions(ctrlOpts);

% Adaptation.
adaptOpts.type  = 'euclidean';          % 'none','euclidean','bregman'
adaptOpts.Gamma = 1e-2 * [36, 12, 12, 12, 8, 8, 12, 0.4, 0.4, 0.4];  % pi = [m,hx,hy,hz,Ixx,Iyy,Izz,Ixy,Ixz,Iyz]
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

% Run the simulation.
sim = fth.sim.SimRunner(cfg);
sim.setup();

runOpts.plotMode     = 'summary';   % 'summary', 'all', or 'none'
runOpts.displayPlots = false;
runOpts.saveSimData  = false;
sim.run(runOpts);
