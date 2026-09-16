function tf = isSPD(X)
%ISSPD True for finite symmetric positive-definite matrices.
tf = isnumeric(X) && ismatrix(X) && all(isfinite(X), 'all') && ...
    norm(X - X.', 'fro') <= 1e-10 * max(1, norm(X, 'fro'));
if ~tf, return; end
[~, p] = chol(0.5 * (X + X.'));
tf = p == 0;
end
