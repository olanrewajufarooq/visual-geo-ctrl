function J = rotInertParams2Matrix(Iparams)
%ROTINERTPARAMS2MATRIX Build 3x3 inertia matrix from params
% Iparams = [Ixx Iyy Izz Ixy Ixz Iyz]
    assert(numel(Iparams) == 6, 'fth:rotInertParams2Matrix: Iparams must have 6 elements');
    Ixx = Iparams(1); Iyy = Iparams(2); Izz = Iparams(3);
    Ixy = Iparams(4); Ixz = Iparams(5); Iyz = Iparams(6);
    J = [ Ixx, Ixy, Ixz;
          Ixy, Iyy, Iyz;
          Ixz, Iyz, Izz ];
end
