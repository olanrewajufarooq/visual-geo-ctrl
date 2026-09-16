function replay3D(run, options)
%REPLAY3D Post-process a saved run as a Robotics Toolbox 3-D animation.
arguments
    run struct
    options.frameStride (1,1) double {mustBePositive, mustBeInteger} = 5
    options.exportFile (1,:) char = ''
end
figure('Name', 'AGC trajectory replay', 'Color', 'w');
ax = axes(); hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal'); view(ax, 3);
xlabel(ax, 'x (m)'); ylabel(ax, 'y (m)'); zlabel(ax, 'z (m)');
actual = squeeze(run.H(1:3,4,:)).'; desired = squeeze(run.Hdesired(1:3,4,:)).';
plot3(ax, actual(:,1), actual(:,2), actual(:,3), 'b-', 'DisplayName', 'actual');
plot3(ax, desired(:,1), desired(:,2), desired(:,3), 'r--', 'DisplayName', 'desired');
legend(ax, 'Location', 'best');
[xx, yy] = meshgrid(linspace(min([actual(:,1); desired(:,1)]) - 1, max([actual(:,1); desired(:,1)]) + 1, 2));
surf(ax, xx, yy, zeros(2), 'FaceAlpha', 0.08, 'EdgeColor', 'none');
vehicle = plot3(ax, NaN, NaN, NaN, 'ko', 'MarkerFaceColor', [0.1 0.3 0.8], 'MarkerSize', 10);
writer = [];
if ~isempty(options.exportFile), writer = VideoWriter(options.exportFile, 'MPEG-4'); open(writer); end
for k = 1:options.frameStride:numel(run.t)
    p = run.H(1:3,4,k); set(vehicle, 'XData', p(1), 'YData', p(2), 'ZData', p(3));
    title(ax, sprintf('t = %.2f s, ||s|| = %.3g', run.t(k), norm(run.s(k,:))));
    drawnow;
    if ~isempty(writer), writeVideo(writer, getframe(gcf)); end
end
if ~isempty(writer), close(writer); end
end
