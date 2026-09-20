function Y = regressor(H, V, Vr, VrDot, gravity, form)
%REGRESSOR Assemble the 6-by-10 reference-model wrench regressor.
%
% Parameter order is pi = [m; h_x; h_y; h_z; I_xx; I_yy; I_zz;
% I_xy; I_xz; I_yz]. The result satisfies
%
%   Y*pi = I*VrDot + C_i(V,I)*Vr + W_g(H,I).

%% Validate and normalize coordinate vectors

V = V(:);
Vr = Vr(:);
VrDot = VrDot(:);
gravity = gravity(:);

validateattributes(H, {'numeric'}, {'real', 'finite', 'size', [4 4]});
validateattributes(V, {'numeric'}, {'real', 'finite', 'numel', 6});
validateattributes(Vr, {'numeric'}, {'real', 'finite', 'numel', 6});
validateattributes(VrDot, {'numeric'}, {'real', 'finite', 'numel', 6});
validateattributes(gravity, {'numeric'}, {'real', 'finite', 'numel', 3});

%% Inertial, Coriolis--gyroscopic, and gravity contributions

YI = inertialBlock(VrDot);
Yc = coriolisBlock(form, V, Vr);
Yg = gravityBlock(H, gravity);

Y = YI + Yc + Yg;
end

function YI = inertialBlock(twist)
%INERTIALBLOCK Return Y_I([alpha; beta]) from the manuscript derivation.

alpha = twist(1:3);
beta = twist(4:6);

% I_b*alpha is linear in [Ixx Iyy Izz Ixy Ixz Iyz].
B = [alpha(1), 0, 0, alpha(2), alpha(3), 0; ...
     0, alpha(2), 0, alpha(1), 0, alpha(3); ...
     0, 0, alpha(3), 0, alpha(1), alpha(2)];

YI = [zeros(3,1), -agc.math.skew(beta), B; ...
      beta,        agc.math.skew(alpha), zeros(3,6)];
end

function Yc = coriolisBlock(form, V, Vr)
%CORIOLISBLOCK Return the explicit 6-by-10 C_i(V,I)*Vr coefficient.

adV = agc.math.adTwist(V);
adVr = agc.math.adTwist(Vr);

switch lower(string(form))
    case "c1"
        % C_1(V,I)Vr = -ad_Vr^* I*V.
        Yc = -adVr.' * inertialBlock(V);
    case "c2"
        % C_2 is the Levi--Civita factorization used in the paper.
        Yc = 0.5 * (inertialBlock(adV * Vr) ...
            - adVr.' * inertialBlock(V) ...
            - adV.' * inertialBlock(Vr));
    otherwise
        error('agc:paper:regressor:UnknownForm', 'form must be c1 or c2.');
end
end

function Yg = gravityBlock(H, gravity)
%GRAVITYBLOCK Return Y_g for the paper's body-frame gravity convention.

gBody = H(1:3,1:3).' * gravity;
Yg = [zeros(3,1), -agc.math.skew(gBody), zeros(3,6); ...
      gBody,        zeros(3,3),             zeros(3,6)];
end
