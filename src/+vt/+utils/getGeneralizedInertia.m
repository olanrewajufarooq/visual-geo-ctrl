function G = getGeneralizedInertia(m, Iparams, CoG)
%GETGENERALIZEDINERTIA Build 6x6 generalized inertia matrix
% Iparams = [Ixx Iyy Izz Ixy Iyz Ixz]
    Ixx = Iparams(1); Iyy = Iparams(2); Izz = Iparams(3);
    Ixy = Iparams(4); Iyz = Iparams(5); Ixz = Iparams(6);

    I_mat = [ Ixx, Ixy, Ixz;
              Ixy, Iyy, Iyz;
              Ixz, Iyz, Izz ];

    G = zeros(6,6);
    G(1:3,1:3) = I_mat;
    G(4:6,4:6) = m * eye(3);
    G(4:6,1:3) = -m * vt.se3.hat3(CoG);
    G(1:3,4:6) =  m * vt.se3.hat3(CoG);
end
