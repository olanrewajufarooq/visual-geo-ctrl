% PLOT_TRAJECTORIES Plot one recorded, preprocessed replay trajectory.
% The input is trajectories/processed/<replayId>.mat, not a simulation run.

%% User settings

replayId = 'lemniscate_01_auto';
samplePeriod = 0.02;
outputDirectory = '';

%% Load the preserved preprocessed replay artifact

startup;
root = agc.io.repositoryRoot();
artifactPath = fullfile(root, 'trajectories', 'processed', [replayId, '.mat']);
if ~isfile(artifactPath)
    error('plot_trajectories:ReplayNotFound', 'No preprocessed replay at %s.', artifactPath);
end
loaded = load(artifactPath, 'traj');
sampler = agc.sim.replayTrajectory(artifactPath);
time = (loaded.traj.t(1):samplePeriod:loaded.traj.t(end)).';
if time(end) < loaded.traj.t(end), time(end + 1,1) = loaded.traj.t(end); end

%% Sample replay pose, body velocity, and body acceleration

n = numel(time);
position = zeros(n,3);
velocity = zeros(n,3);
angularVelocity = zeros(n,3);
acceleration = zeros(n,3);
angularAcceleration = zeros(n,3);
for k = 1:n
    desired = sampler(time(k));
    position(k,:) = desired.H(1:3,4).';
    angularVelocity(k,:) = desired.V(1:3).';
    velocity(k,:) = desired.V(4:6).';
    angularAcceleration(k,:) = desired.Vdot(1:3).';
    acceleration(k,:) = desired.Vdot(4:6).';
end

%% Render geometry and recorded kinematic channels

figureHandle = figure('Name', ['Preprocessed replay: ', replayId], 'Color', 'w', ...
    'Position', [100, 100, 1400, 800]);
layout = tiledlayout(3, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, ['Preprocessed replay: ', replayId], 'Interpreter', 'none');

axPath = nexttile(layout, 1, [3, 1]);
plot3(axPath, position(:,1), position(:,2), position(:,3), 'k-', 'LineWidth', 1.4);
hold(axPath, 'on');
plot3(axPath, position(1,1), position(1,2), position(1,3), 'go', 'MarkerFaceColor', 'g');
plot3(axPath, position(end,1), position(end,2), position(end,3), 'rd', 'MarkerFaceColor', 'r');
grid(axPath, 'on'); axis(axPath, 'equal'); view(axPath, 3);
xlabel(axPath, 'x (m)'); ylabel(axPath, 'y (m)'); zlabel(axPath, 'z (m)');
legend(axPath, {'path', 'start', 'end'}, 'Location', 'best');

axPosition = nexttile(layout); plot(axPosition, time, position, 'LineWidth', 1.2); grid(axPosition, 'on'); title(axPosition, 'Position'); ylabel(axPosition, 'm'); legend(axPosition, {'x','y','z'}, 'Location', 'best');
axVelocity = nexttile(layout); plot(axVelocity, time, velocity, 'LineWidth', 1.2); grid(axVelocity, 'on'); title(axVelocity, 'Body linear velocity'); ylabel(axVelocity, 'm/s'); legend(axVelocity, {'v_x','v_y','v_z'}, 'Location', 'best');
axAcceleration = nexttile(layout); plot(axAcceleration, time, acceleration, 'LineWidth', 1.2); grid(axAcceleration, 'on'); title(axAcceleration, 'Body linear acceleration'); ylabel(axAcceleration, 'm/s^2'); legend(axAcceleration, {'a_x','a_y','a_z'}, 'Location', 'best');
axAngularVelocity = nexttile(layout); plot(axAngularVelocity, time, angularVelocity, 'LineWidth', 1.2); grid(axAngularVelocity, 'on'); title(axAngularVelocity, 'Body angular velocity'); ylabel(axAngularVelocity, 'rad/s'); legend(axAngularVelocity, {'omega_x','omega_y','omega_z'}, 'Location', 'best');
axAngularAcceleration = nexttile(layout); plot(axAngularAcceleration, time, angularAcceleration, 'LineWidth', 1.2); grid(axAngularAcceleration, 'on'); title(axAngularAcceleration, 'Body angular acceleration'); ylabel(axAngularAcceleration, 'rad/s^2'); legend(axAngularAcceleration, {'alpha_x','alpha_y','alpha_z'}, 'Location', 'best');

for ax = [axPosition, axVelocity, axAcceleration, axAngularVelocity, axAngularAcceleration]
    xlabel(ax, 'time (s)');
end

%% Optionally export a static reference-trajectory figure

if ~isempty(outputDirectory)
    if ~isfolder(outputDirectory), mkdir(outputDirectory); end
    exportgraphics(figureHandle, fullfile(outputDirectory, [replayId, '_trajectory.png']), 'Resolution', 300);
    exportgraphics(figureHandle, fullfile(outputDirectory, [replayId, '_trajectory.pdf']), 'ContentType', 'vector');
end
