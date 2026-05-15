function S = symOfMat(A)
%SYMOFMAT Symmetric part of a square matrix: S = (A + A') / 2.
    S = 0.5 * (A + A');
end
