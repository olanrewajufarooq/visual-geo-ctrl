function Vdot = acceleration(plant, state, wrench)
%ACCELERATION Evaluate Robotics Toolbox forward dynamics in body coordinates.

%% Validate the paper-state inputs

H = state.H; V = state.V(:); wrench = wrench(:);
validateattributes(H, {'numeric'}, {'real', 'finite', 'size', [4 4]});
validateattributes(V, {'numeric'}, {'real', 'finite', 'numel', 6});
validateattributes(wrench, {'numeric'}, {'real', 'finite', 'numel', 6});

%% Convert SE(3) state and body wrench for rigidBodyTree

% The floating joint uses [quaternion position] configuration coordinates.
q = [rotm2quat(H(1:3,1:3)), H(1:3,4).'];

% externalForce expresses the paper body wrench in the Toolbox body frame.
fext = externalForce(plant.robot, plant.bodyName, wrench.', q);
Vdot = forwardDynamics(plant.robot, q, V.', zeros(1,6), fext).';
end
