function plant = floatingBody(pi, gravity)
%FLOATINGBODY Construct the sole Robotics System Toolbox floating-base plant.
%
% GRAVITY is the physical world-frame acceleration expected by rigidBodyTree.
pi = pi(:); gravity = gravity(:);
validateattributes(pi, {'numeric'}, {'real', 'finite', 'numel', 10});
validateattributes(gravity, {'numeric'}, {'real', 'finite', 'numel', 3});

%% Configure the world and one floating rigid body

robot = rigidBodyTree(DataFormat='row');
robot.Gravity = gravity.';

% RigidBody uses [Ixx Iyy Izz Ixy Ixz Iyz], the same order as pi(5:10).
body = rigidBody('vehicle');
body.Joint = rigidBodyJoint('floating', 'floating');
body.Mass = pi(1);
body.CenterOfMass = (pi(2:4) ./ pi(1)).';
body.Inertia = [pi(5:7); pi(8:10)].';
addBody(robot, body, robot.BaseName);

%% Return a small immutable plant handle struct

plant = struct('robot', robot, 'gravity', gravity, 'bodyName', 'vehicle');
end
