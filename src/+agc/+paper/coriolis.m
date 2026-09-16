function value = coriolis(form, V, I6, U)
%CORIOLIS Evaluate the paper C_i(V,I)U action, not a matrix surrogate.
V = V(:); U = U(:);
validateattributes(I6, {'numeric'}, {'real', 'finite', 'size', [6 6]});
switch lower(string(form))
    case "c1"
        value = -agc.math.adTwist(U).' * (I6 * V);
    case "c2"
        value = 0.5 * (I6 * agc.math.adTwist(V) * U ...
            - agc.math.adTwist(U).' * (I6 * V) ...
            - agc.math.adTwist(V).' * (I6 * U));
    otherwise
        error('agc:paper:coriolis:UnknownForm', 'form must be c1 or c2.');
end
end
