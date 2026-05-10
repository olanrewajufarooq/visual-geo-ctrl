function S = sym(A)
%SYM Symmetric part of a square matrix: S = (A + A') / 2.
%   Counterpart to skew, which returns the skew-symmetric part (A - A') / 2.
%   Input:
%     A - n×n matrix.
%   Output:
%     S - n×n symmetric matrix, S = 0.5*(A + A').
    S = 0.5 * (A + A');
end
