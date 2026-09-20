function loadedPi = compoundPi(barePi, payload)
%COMPOUNDPI Add an aligned cuboid payload to body-origin inertial parameters.
%
% PAYLOAD defines its mass, dimensions, and center in the vehicle body frame.
% The returned 10-by-1 vector uses pi = [m; h; Ixx; Iyy; Izz; Ixy; Ixz; Iyz]
% about the same vehicle body origin as barePi.

barePi = barePi(:);
validateattributes(barePi, {'numeric'}, {'real', 'finite', 'numel', 10});
payload = normalizePayload(payload);

m = payload.mass;
r = payload.center;
d = payload.dimensions;
payloadInertiaAtCom = m / 12 * diag([d(2)^2 + d(3)^2, ...
    d(1)^2 + d(3)^2, d(1)^2 + d(2)^2]);
payloadInertiaAtBody = payloadInertiaAtCom + m * ((r.' * r) * eye(3) - r * r.');
bareInertia = [barePi(5), barePi(8), barePi(9); ...
    barePi(8), barePi(6), barePi(10); ...
    barePi(9), barePi(10), barePi(7)];
loadedInertia = bareInertia + payloadInertiaAtBody;
loadedPi = [barePi(1) + m; barePi(2:4) + m * r; loadedInertia(1,1); ...
    loadedInertia(2,2); loadedInertia(3,3); loadedInertia(1,2); ...
    loadedInertia(1,3); loadedInertia(2,3)];
end

function payload = normalizePayload(payload)
if ~isstruct(payload) || ~all(isfield(payload, {'mass', 'dimensions', 'center'}))
    error('agc:plant:compoundPi:Payload', ...
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
