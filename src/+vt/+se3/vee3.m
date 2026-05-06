function w = vee3(S)
%VEE3 Isomorphism so(3) -> R^3: skew-symmetric matrix to vector.
%   Inverse of hat3: vee3(hat3(w)) = w.
    w = [S(3,2); S(1,3); S(2,1)];
end
