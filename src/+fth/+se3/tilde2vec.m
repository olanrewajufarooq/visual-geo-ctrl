function v = tilde2vec(X)
%TILDE2VEC Map a tilde matrix to its vector coordinates.
%   3x3 input: so(3) -> R^3
%   4x4 input: se(3) -> R^6, with twist ordering [omega; v]
    if all(size(X) == [3 3])
        v = [X(3,2); X(1,3); X(2,1)];
    elseif all(size(X) == [4 4])
        v = [X(3,2); X(1,3); X(2,1); X(1:3,4)];
    else
        error('fth:tilde2vec:InvalidDimension', ...
              'Input must be a 3x3 or 4x4 matrix.');
    end
end
