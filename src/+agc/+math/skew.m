function X = skew(x)
%SKEW Cross-product matrix: skew(x) * y == cross(x,y).
x = x(:);
validateattributes(x, {'numeric'}, {'real', 'finite', 'numel', 3});
X = [0, -x(3), x(2); x(3), 0, -x(1); -x(2), x(1), 0];
end
