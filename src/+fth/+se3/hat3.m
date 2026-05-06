function S = hat3(w)
%HAT3 Isomorphism R^3 -> so(3): vector to skew-symmetric matrix.
%   hat([w1; w2; w3]) = [0, -w3, w2; w3, 0, -w1; -w2, w1, 0].
%   Satisfies hat(a) * b = cross(a, b).
    S = [  0,   -w(3),  w(2);
          w(3),  0,   -w(1);
         -w(2), w(1),  0 ];
end
