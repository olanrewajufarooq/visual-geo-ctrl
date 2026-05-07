%PLOT_TRAJECTORIES Sample and plot reference trajectories.
%   Saves one detail figure per trajectory and a summary figure to
%   results/trajectories/. Pass a cell array to plot a subset.

clear; close all;
startup;

fth.plot.TrajPlotter.run();

% To plot a subset, pass trajectory names:
%   fth.plot.TrajPlotter.run({'circle', 'infinity'})
