function varargout = errors(action, varargin)
%ERRORS Evaluate paper tracking-error covectors and their derivatives.

%% Dispatch the requested error operation

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
%POTENTIALDERIVATIVE Differentiate e_H along left-trivialized error motion.

%% Error-state kinematics

R = He(1:3,1:3); p = He(1:3,4); Ve = Ve(:);
omega = Ve(1:3); v = Ve(4:6);
Rdot = R * agc.math.skew(omega);

%% Rotational and translational covector derivatives

eRdot = agc.math.unskew(0.5 * (KR * Rdot - Rdot.' * KR));
ep = R.' * Kxi * p;
epdot = -agc.math.skew(omega) * ep + R.' * Kxi * R * v;
eHdot = [eRdot; epdot];
end
end

function [eH, psi] = potential(He, KR, Kxi)
%POTENTIAL Return e_H=beta_He^*dPsi and the configuration potential Psi.

%% Validate the SE(3) error configuration and design weights

validateattributes(He, {'numeric'}, {'real', 'finite', 'size', [4 4]});
validateattributes(KR, {'numeric'}, {'real', 'finite', 'size', [3 3]});
validateattributes(Kxi, {'numeric'}, {'real', 'finite', 'size', [3 3]});
R = He(1:3,1:3); p = He(1:3,4);

%% Evaluate rotational and translational terms

eR = agc.math.unskew(0.5 * (KR * R - R.' * KR));
ep = R.' * Kxi * p;
eH = [eR; ep];
psi = 0.5 * trace(KR * (eye(3) - R)) + 0.5 * p.' * Kxi * p;
end
