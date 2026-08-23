%PLOT_ANALYTIC_TRAJECTORIES Sample and plot analytic reference trajectories.
%   Saves figures to results/trajectories/<timestamp>/.

clear; close all;
startup;

opts.period                   = 90;    % path cycle [s]
opts.goToHoverDuration        = 30;    % hover climb before path starts [s]
opts.goToHoverBeforePathStarts = true;
opts.dt                       = 0.01;  % sampling interval [s]
fth.plot.TrajPlotter.run(opts);

% ----- Plot all with defaults ---------

% Plot all trajectories with default parameters.
% fth.plot.TrajPlotter.run(struct('mode', 'analytic'));

% ---- Customization (uncomment and edit as needed) --------------------
%
% % Scalar opts apply to all trajectories:
% opts.names                    = {'circle', 'lissajous3d'};
% opts.scale                    = 3;                    % path scale [m]
% opts.altitude                 = 8;                    % hover altitude [m]
% opts.period                   = 60;                   % path cycle [s]
% opts.goToHoverDuration        = 30;                   % hover climb [s]
% opts.goToHoverBeforePathStarts = true;
% opts.dt                       = 0.01;
% fth.plot.TrajPlotter.run(opts);
%
% % Per-trajectory vector opts (one value per name):
% opts.names                    = {'circle', 'lissajous3d'};
% opts.scale                    = [3, 5];               % one per trajectory
% opts.period                   = [40, 60];             % one per trajectory
% opts.goToHoverDuration        = [20, 30];             % one per trajectory
% opts.goToHoverBeforePathStarts = [true, false];       % one per trajectory
% fth.plot.TrajPlotter.run(opts);
