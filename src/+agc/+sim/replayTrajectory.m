function sampler = replayTrajectory(filePath)
%REPLAYTRAJECTORY Load one preserved replay artifact as a paper-state sampler.
loaded = load(filePath, 'traj');
if ~isfield(loaded, 'traj')
    error('agc:sim:replayTrajectory:MissingTraj', 'Replay artifact must contain traj.');
end
traj = loaded.traj;
required = {'t', 'p', 'v_b', 'a_b', 'omega_b', 'alpha_b', 'R'};
for k = 1:numel(required)
    if ~isfield(traj, required{k})
        error('agc:sim:replayTrajectory:MissingField', 'traj.%s is required.', required{k});
    end
end
t = traj.t(:);
sampler = @(time) sample(t, traj, time);
end

function desired = sample(t, traj, time)
time = min(max(time, t(1)), t(end));
p = interp1(t, traj.p, time, 'linear').';
v = interp1(t, traj.v_b, time, 'linear').';
a = interp1(t, traj.a_b, time, 'linear').';
omega = interp1(t, traj.omega_b, time, 'linear').';
alpha = interp1(t, traj.alpha_b, time, 'linear').';
upper = find(t >= time, 1, 'first');
if upper == 1
    R = traj.R(:,:,1);
else
    lower = upper - 1;
    ratio = (time - t(lower)) / (t(upper) - t(lower));
    q0 = rotm2quat(traj.R(:,:,lower)); q1 = rotm2quat(traj.R(:,:,upper));
    if dot(q0, q1) < 0, q1 = -q1; end
    q = (1 - ratio) * q0 + ratio * q1; q = q / norm(q);
    R = quat2rotm(q);
end
desired = struct('H', [R, p; 0, 0, 0, 1], 'V', [omega; v], 'Vdot', [alpha; a]);
end
