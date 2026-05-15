function S = skewOfMat(A)
%SKEWOFMAT Skew-symmetric part of a square matrix: S = (A - A') / 2.
    S = 0.5 * (A - A');
end
