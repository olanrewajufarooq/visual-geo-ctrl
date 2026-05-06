%RUN_ADAPTIVE_DEMO Adaptive control simulation with payload drop.
%   Demonstrates online parameter adaptation and payload change.

% Clean workspace and load project paths.
clear; close all;
startup;

% Build a fresh configuration with defaults.
cfg = vt.config.Config();

% Scenario duration in seconds.
duration = 30;

% Timing parameters.
cfg.setSimParams(0.005, duration);          % sim dt, duration
cfg.setAdaptationParams(0.005);             % adaptation dt
cfg.setControlParams(0.01);                 % control dt

% Reference trajectory setup.
cfg.setTrajectory( ...
   'infinity3dmod', ... % 'hover','circle','infinity','infinity3d','infinity3dmod','lissajous3d','helix3d','poly3d','takeoffland'
   2.25, ... % cycle count
   true); % Start with hover (boolean)
cfg.setTrajectoryMethod('precomputed');      % 'precomputed','modelreference'

% Controller and potential selection.
cfg.setController('Feedforward');          % 'PD','FeedLin','Feedforward'
cfg.setPotentialType('liealgebra');        % 'liealgebra' or 'separate'
cfg.setAdaptation('euclidean');            % 'none','euclidean','geo-aware'

% Gain configuration
cfg.setKpGains([5.5, 5.5, 5.5, 5.5, 5.5, 5.5]');          % proportional gains for position and attitude
cfg.setKdGains([2.05, 2.05, 2.05, 2.05, 2.05, 2.05]');      % derivative gains for position and attitude
cfg.setAdaptiveGains(1e-2 * [8,   8,  12, 0.4, 0.4, 0.4, 36,  12,  12,  12]);

% Live visualization preferences.
cfg.enableLiveView(true);
cfg.setLiveSummary(true);
cfg.setLiveUpdateRate(500);
cfg.setLiveUrdfEmbedding(true);
cfg.setPlotLayout('column-major');

% Payload schedule (mass drop event).
cfg.setPayloadScenario( ...
   1.5, ...                    % mass
   [0.115; 0.05; -0.05], ...    % comOffset
   2*duration/3 ...             % dropTime
);
cfg.setEstimateInitialization('random');
cfg.done();

% Run the simulation, save plots silently, and skip sim_data.mat.
sim = vt.sim.SimRunner(cfg);
sim.setup();
sim.run( ...
   'summary', ...       % plotting mode: 'summary', 'all', or 'none'
   false, ...           % display plots while saving
   false ...            % save sim_data.mat
); 
