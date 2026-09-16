function Y = regressor(H, V, Vr, VrDot, gravity, form)
%REGRESSOR Exact 6-by-10 reference-model wrench regressor.
% The basis construction is intentionally direct: every column evaluates
% the same paper equation used by the controller, preventing convention drift.
V = V(:); Vr = Vr(:); VrDot = VrDot(:); gravity = gravity(:);
validateattributes(H, {'numeric'}, {'real', 'finite', 'size', [4 4]});
validateattributes(V, {'numeric'}, {'real', 'finite', 'numel', 6});
validateattributes(Vr, {'numeric'}, {'real', 'finite', 'numel', 6});
validateattributes(VrDot, {'numeric'}, {'real', 'finite', 'numel', 6});
validateattributes(gravity, {'numeric'}, {'real', 'finite', 'numel', 3});
Y = zeros(6, 10);
for k = 1:10
    pi = zeros(10,1); pi(k) = 1;
    I6 = agc.math.inertiaFromPi(pi);
    Y(:,k) = I6 * VrDot + agc.paper.coriolis(form, V, I6, Vr) + ...
        gravityWrench(H, pi, gravity);
end
end

function Wg = gravityWrench(H, pi, gravity)
gBody = H(1:3,1:3).' * gravity;
Wg = [agc.math.skew(pi(2:4)) * gBody; pi(1) * gBody];
end
