function value = coriolis(form, V, I6, U)
%CORIOLIS Evaluate the paper C_i(V,I)U action, not a stored matrix.
%
% V is the differentiation direction and U is the tangent vector acted on.
V = V(:); U = U(:);
validateattributes(I6, {'numeric'}, {'real', 'finite', 'size', [6 6]});

%% Select the metric-compatible factorization

switch lower(string(form))
    case "c1"
        % C_1(V,I)U = -ad_U^*(I*V).
        value = -agc.math.adTwist(U).' * (I6 * V);
    case "c2"
        % C_2 is the left-trivialized Levi--Civita factorization.
        value = 0.5 * (I6 * agc.math.adTwist(V) * U ...
            - agc.math.adTwist(U).' * (I6 * V) ...
            - agc.math.adTwist(V).' * (I6 * U));
    otherwise
        error('agc:paper:coriolis:UnknownForm', 'form must be c1 or c2.');
end
end
