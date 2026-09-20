function plant = floatingBody(pi, gravity, payload)
%FLOATINGBODY Construct a floating UAV, optionally with a fixed payload body.
%
% GRAVITY is the physical world-frame acceleration expected by rigidBodyTree.
if nargin < 3, payload = []; end
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

if ~isempty(payload)
    payload = validatePayload(payload);
    payloadBody = rigidBody('payload');
    payloadBody.Joint = rigidBodyJoint('payload_fixed', 'fixed');
    setFixedTransform(payloadBody.Joint, trvec2tform(payload.center.'));
    payloadBody.Mass = payload.mass;
    payloadBody.CenterOfMass = [0, 0, 0];
    payloadBody.Inertia = cuboidInertia(payload.mass, payload.dimensions).';
    addBody(robot, payloadBody, body.Name);
end

%% Return a small immutable plant handle struct

plant = struct('robot', robot, 'gravity', gravity, 'bodyName', 'vehicle');
end

function payload = validatePayload(payload)
%VALIDATEPAYLOAD Normalize a body-frame cuboid payload description.

if ~isstruct(payload) || ~all(isfield(payload, {'mass', 'dimensions', 'center'}))
    error('agc:plant:floatingBody:Payload', ...
        'payload must define mass, dimensions, and center.');
end
payload.mass = payload.mass(:);
payload.dimensions = payload.dimensions(:);
payload.center = payload.center(:);
validateattributes(payload.mass, {'numeric'}, {'real', 'finite', 'positive', 'numel', 1});
validateattributes(payload.dimensions, {'numeric'}, {'real', 'finite', 'positive', 'numel', 3});
validateattributes(payload.center, {'numeric'}, {'real', 'finite', 'numel', 3});
payload.mass = payload.mass(1);
end

function inertia = cuboidInertia(mass, dimensions)
%CUBOIDINERTIA Principal inertia of an aligned cuboid about its center.

dimensions = dimensions(:);
inertia = mass / 12 * [dimensions(2)^2 + dimensions(3)^2; ...
    dimensions(1)^2 + dimensions(3)^2; ...
    dimensions(1)^2 + dimensions(2)^2; 0; 0; 0];
end
