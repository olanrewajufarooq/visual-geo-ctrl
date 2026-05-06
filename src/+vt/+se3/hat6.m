function se3mat = hat6(V)
%HAT6 Isomorphism R^6 -> se(3): twist vector to 4x4 Lie algebra element.
%   hat6([omega; v]) = [hat3(omega), v; 0, 0].
%
%   Input:  V - 6x1 twist [angular; linear].
%   Output: se3mat - 4x4 se(3) matrix.
    w = V(1:3);
    v = V(4:6);
    se3mat = [vt.se3.hat3(w), v; 0 0 0 0];
end
