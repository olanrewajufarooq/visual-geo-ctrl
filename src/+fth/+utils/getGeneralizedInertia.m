function G = getGeneralizedInertia(m, Iparams, CoG)
%GETGENERALIZEDINERTIA Build 6x6 generalized inertia matrix.
%   Iparams = [Ixx Iyy Izz Ixy Iyz Ixz]
%   CoG: 3x1 offset of center of mass from body frame origin, in body frame [m].
    assert(isscalar(m) && m > 0, 'fth:getGeneralizedInertia: m must be a positive scalar');
    assert(numel(Iparams) == 6, 'fth:getGeneralizedInertia: Iparams must have 6 elements');
    assert(numel(CoG) == 3, 'fth:getGeneralizedInertia: CoG must be a 3-element vector');

    Ixx = Iparams(1); Iyy = Iparams(2); Izz = Iparams(3);
    Ixy = Iparams(4); Iyz = Iparams(5); Ixz = Iparams(6);

    I_mat = [ Ixx, Ixy, Ixz;
              Ixy, Iyy, Iyz;
              Ixz, Iyz, Izz ];

    G = zeros(6,6);
    G(1:3,1:3) = I_mat;
    G(4:6,4:6) = m * eye(3);
    G(4:6,1:3) = -m * fth.se3.hat3(CoG);
    G(1:3,4:6) =  m * fth.se3.hat3(CoG);
end
