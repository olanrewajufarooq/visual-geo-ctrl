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
    {'circle', 'infinity3dmod', 'lissajous3d', 'helix3d', 'poly3d'}, ...
    1.25, ... % cycle count
    true); % Start with hover (boolean)

% Controller and potential selection.
cfg.setController('Feedforward');          % 'PD','FeedLin','Feedforward'
cfg.setPotentialType('liealgebra');        % 'liealgebra' or 'separate'

cfg.setAdaptation('euclidean');            % 'none','euclidean','geo-aware'
cfg.setAdaptiveGains(1e-2 * [ ...
      0,   0,   0,   0,   0,   0,  0,   0,   0,   0; ...
      8,   8,  12, 0.4, 0.4, 0.4, 36,  12,  12,  12; ...
    360, 360, 360,  40,  40,  40, 72,  12,  12,  12; ...
      8,   8,  12, 0.4, 0.4, 0.4, 36, 120, 120, 120]); % rows: Run 1, Run 2, Run 3, Run 4

% Payload schedule (mass drop event).
cfg.setPayloadScenario( ...
   1.5, ...                    % mass
   [0.115; 0.05; -0.05], ...    % comOffset
   2*duration/3 ...             % dropTime
);

% Options: 'nominal', 'true', 'fixed', 'fixed-higher', 'random', or a 10x1/1x10 custom theta vector for fixed initialization
cfg.setEstimateInitialization('nominal');

cfg.done();

% Run the simulation, save plots silently, and skip sim_data.mat.
sim = vt.sim.SimRunner(cfg);
sim.setup();
sim.run( ...
   'all', ...          % plotting mode: 'summary', 'all', or 'none'
   false, ...          % display plots while saving
   false ...           % save sim_data.mat
); 
