function V = vee6(se3mat)
%VEE6 Isomorphism se(3) -> R^6: Lie algebra element to twist vector.
%   Inverse of hat6: vee6(hat6(V)) = V.
%
%   Input:  se3mat - 4x4 se(3) matrix.
%   Output: V - 6x1 twist [angular; linear].
    V = [se3mat(3,2); se3mat(1,3); se3mat(2,1); se3mat(1:3,4)];
end
