function value = adaptation(action, varargin)
%ADAPTATION Discrete-time updates for the paper estimators.

%% Dispatch the requested estimator operation

switch lower(string(action))
    case "bregman-step"
        value = bregmanStep(varargin{:});
    otherwise
        error('agc:paper:adaptation:UnknownAction', 'Unknown adaptation action.');
end
end

function Jnext = bregmanStep(J, G, gamma, dt)
%BREGMANSTEP SPD-preserving affine-invariant pseudo-inertia update.

%% Validate the current pseudo-inertia and numerical step

validateattributes(J, {'numeric'}, {'real', 'finite', 'size', [4 4]});
validateattributes(G, {'numeric'}, {'real', 'finite', 'size', [4 4]});
validateattributes(gamma, {'numeric'}, {'real', 'finite', 'positive', 'scalar'});
validateattributes(dt, {'numeric'}, {'real', 'finite', 'positive', 'scalar'});
if ~agc.math.isSPD(J)
    error('agc:paper:adaptation:NonSPD', 'J must be symmetric positive definite.');
end

%% Evaluate the symmetric congruence-exponential update

[Q, D] = eig(0.5 * (J + J.'));
Jhalf = Q * diag(sqrt(diag(D))) * Q.';
Gsym = 0.5 * (G + G.');

% Congruence by J^(1/2) and expm of a symmetric matrix preserve SPD.
Jnext = Jhalf * expm(-gamma * dt * Jhalf * Gsym * Jhalf) * Jhalf;
Jnext = 0.5 * (Jnext + Jnext.');
end
