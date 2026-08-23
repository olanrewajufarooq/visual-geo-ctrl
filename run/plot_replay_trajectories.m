%PLOT_REPLAY_TRAJECTORIES Plot processed replay trajectories.

clear; close all;
startup;

opts.mode = 'replay';
opts.names = {
    'ellipse_01_auto',
    'lemniscate_01_auto',
    'RATM_01_auto'
};  % {} = all replay flights
fth.plot.TrajPlotter.run(opts);

% Plot one replay by manifest id or label:
% opts.names = {'ellipse_01_Auto'};  % artifact name, .mat extension optional
% opts.names = {'Ellipse 01 Autonomous'};  % manifest label
% fth.plot.TrajPlotter.run(opts);
