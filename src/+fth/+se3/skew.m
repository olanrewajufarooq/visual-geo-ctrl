function S = skew(A)
%SKEW Skew-symmetric part of a square matrix: S = (A - A') / 2.
%   Input:
%     A - n×n matrix.
%   Output:
%     S - n×n skew-symmetric matrix, S = 0.5*(A - A').
    S = 0.5 * (A - A');
end
