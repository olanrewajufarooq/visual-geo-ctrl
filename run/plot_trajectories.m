%PLOT_TRAJECTORIES Sample and plot reference trajectories.
%   Saves figures to results/trajectories/<timestamp>/.

clear; close all;
startup;

opts.duration = 60;                        % time horizon [s]
opts.goToHoverBeforePathStarts = false;    % skip hover phase
opts.dt       = 0.001;                      % sampling interval [s]
fth.plot.TrajPlotter.run(opts);

% ----- Plot all ---------

% Plot all trajectories with default parameters.
% fth.plot.TrajPlotter.run();

% ---- Customization (uncomment and edit as needed) --------------------
%
% opts.names    = {'circle', 'infinity'};    % subset of trajectories
% opts.scale    = 3;                         % path scale [m]
% opts.altitude = 8;                         % hover altitude [m]
% opts.duration = 20;                        % time horizon [s]
% opts.goToHoverBeforePathStarts = false;    % skip hover phase
% opts.dt       = 0.01;                      % sampling interval [s]
% fth.plot.TrajPlotter.run(opts);
