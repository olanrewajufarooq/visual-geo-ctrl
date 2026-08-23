%PLOT_REPLAY_TRAJECTORIES Plot processed replay trajectories.

clear; close all;
startup;

opts.mode = 'replay';
opts.names = {};  % {} = all replay flights
fth.plot.TrajPlotter.run(opts);

% Plot one replay by manifest id or label:
% opts.names = {'flight_01a_ellipse'};
% opts.names = {'Ellipse 01 Autonomous'};
% fth.plot.TrajPlotter.run(opts);
