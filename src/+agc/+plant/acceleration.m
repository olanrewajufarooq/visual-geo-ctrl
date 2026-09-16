function Vdot = acceleration(plant, state, wrench)
%ACCELERATION Evaluate Robotics Toolbox forward dynamics in body coordinates.
H = state.H; V = state.V(:); wrench = wrench(:);
validateattributes(H, {'numeric'}, {'real', 'finite', 'size', [4 4]});
validateattributes(V, {'numeric'}, {'real', 'finite', 'numel', 6});
validateattributes(wrench, {'numeric'}, {'real', 'finite', 'numel', 6});
q = [rotm2quat(H(1:3,1:3)), H(1:3,4).'];
fext = externalForce(plant.robot, plant.bodyName, wrench.', q);
Vdot = forwardDynamics(plant.robot, q, V.', zeros(1,6), fext).';
end
