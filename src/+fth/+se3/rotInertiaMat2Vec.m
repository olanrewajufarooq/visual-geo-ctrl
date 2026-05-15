function I_vec = rotInertiaMat2Vec(J)
%ROTINERTIAMAT2VEC Vectorize 3x3 rotational inertia matrix.
%   Output ordering: [Ixx; Iyy; Izz; Ixy; Ixz; Iyz]
    assert(all(size(J) == [3 3]), 'fth:rotInertiaMat2Vec: J must be 3x3');
    I_vec = [J(1,1); J(2,2); J(3,3); J(1,2); J(1,3); J(2,3)];
end
