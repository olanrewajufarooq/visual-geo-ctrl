function X = vec2tilde(v)
%VEC2TILDE Map a 3-vector or 6-vector to its tilde matrix.
%   3-vector: R^3 -> so(3)
%   6-vector: R^6 -> se(3), with twist ordering [omega; v]
    v = v(:);
    switch numel(v)
        case 3
            X = [  0,   -v(3),  v(2);
                  v(3),  0,   -v(1);
                 -v(2), v(1),  0 ];
        case 6
            X = [fth.se3.vec2tilde(v(1:3)), v(4:6); 0 0 0 0];
        otherwise
            error('fth:vec2tilde:InvalidDimension', ...
                  'Input must have 3 or 6 elements.');
    end
end
