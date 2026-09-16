function varargout = errors(action, varargin)
%ERRORS Paper tracking-error and potential calculations.
switch lower(string(action))
    case "potential"
        [eH, psi] = potential(varargin{:});
        varargout = {eH, psi};
    case "potential-derivative"
        varargout = {potentialDerivative(varargin{:})};
    otherwise
        error('agc:paper:errors:UnknownAction', 'Unknown errors action.');
end

function eHdot = potentialDerivative(He, Ve, KR, Kxi)
R = He(1:3,1:3); p = He(1:3,4); Ve = Ve(:);
omega = Ve(1:3); v = Ve(4:6);
Rdot = R * agc.math.skew(omega);
eRdot = agc.math.unskew(0.5 * (KR * Rdot - Rdot.' * KR));
ep = R.' * Kxi * p;
epdot = -agc.math.skew(omega) * ep + R.' * Kxi * R * v;
eHdot = [eRdot; epdot];
end
end

function [eH, psi] = potential(He, KR, Kxi)
validateattributes(He, {'numeric'}, {'real', 'finite', 'size', [4 4]});
validateattributes(KR, {'numeric'}, {'real', 'finite', 'size', [3 3]});
validateattributes(Kxi, {'numeric'}, {'real', 'finite', 'size', [3 3]});
R = He(1:3,1:3); p = He(1:3,4);
eR = agc.math.unskew(0.5 * (KR * R - R.' * KR));
ep = R.' * Kxi * p;
eH = [eR; ep];
psi = 0.5 * trace(KR * (eye(3) - R)) + 0.5 * p.' * Kxi * p;
end
