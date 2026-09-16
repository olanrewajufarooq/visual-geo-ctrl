function A = adTwist(V)
%ADTWIST Lie-bracket matrix on se(3), for V=[omega; v].
V = V(:);
validateattributes(V, {'numeric'}, {'real', 'finite', 'numel', 6});
A = [agc.math.skew(V(1:3)), zeros(3); agc.math.skew(V(4:6)), agc.math.skew(V(1:3))];
end
