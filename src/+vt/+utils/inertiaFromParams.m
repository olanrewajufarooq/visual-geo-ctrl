function J = inertiaFromParams(Iparams)
%INERTIAFROMPARAMS Build 3x3 inertia matrix from params
% Iparams = [Ixx Iyy Izz Ixy Iyz Ixz]
    Ixx = Iparams(1); Iyy = Iparams(2); Izz = Iparams(3);
    Ixy = Iparams(4); Iyz = Iparams(5); Ixz = Iparams(6);
    J = [ Ixx, Ixy, Ixz;
          Ixy, Iyy, Iyz;
          Ixz, Iyz, Izz ];
end
