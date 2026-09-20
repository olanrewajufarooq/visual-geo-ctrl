function x = unskew(X)
%UNSKEW Inverse coordinate map for a 3-by-3 skew-symmetric matrix.
validateattributes(X, {'numeric'}, {'real', 'finite', 'size', [3 3]});
x = [X(3,2); X(1,3); X(2,1)];
end
