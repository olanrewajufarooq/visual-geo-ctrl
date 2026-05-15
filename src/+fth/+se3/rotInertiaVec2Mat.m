function J = rotInertiaVec2Mat(I_vec)
%ROTINERTIAVEC2MAT Build 3x3 rotational inertia matrix from vector.
%   I_vec = [Ixx; Iyy; Izz; Ixy; Ixz; Iyz]
    assert(numel(I_vec) == 6, 'fth:rotInertiaVec2Mat: I_vec must have 6 elements');
    Ixx = I_vec(1); Iyy = I_vec(2); Izz = I_vec(3);
    Ixy = I_vec(4); Ixz = I_vec(5); Iyz = I_vec(6);
    J = [ Ixx, Ixy, Ixz;
          Ixy, Iyy, Iyz;
          Ixz, Iyz, Izz ];
end
