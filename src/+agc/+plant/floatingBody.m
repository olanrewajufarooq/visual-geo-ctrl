function plant = floatingBody(pi, gravity)
%FLOATINGBODY Construct the sole Robotics System Toolbox plant model.
pi = pi(:); gravity = gravity(:);
validateattributes(pi, {'numeric'}, {'real', 'finite', 'numel', 10});
validateattributes(gravity, {'numeric'}, {'real', 'finite', 'numel', 3});
robot = rigidBodyTree(DataFormat='row');
robot.Gravity = gravity.';
body = rigidBody('vehicle');
body.Joint = rigidBodyJoint('floating', 'floating');
body.Mass = pi(1);
body.CenterOfMass = (pi(2:4) ./ pi(1)).';
body.Inertia = [pi(5:7); pi(8:10)].';
addBody(robot, body, robot.BaseName);
plant = struct('robot', robot, 'gravity', gravity, 'bodyName', 'vehicle');
end
