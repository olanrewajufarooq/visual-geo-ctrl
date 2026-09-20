function [W, diagnostics, estimateNext] = controller(state, desired, cfg, estimate, dtAdapt)
%CONTROLLER Evaluate the paper-aligned body-wrench tracking controller.
%
% STATE and DESIRED use left-trivialized body velocities [omega; v].
% ESTIMATE is pi for nominal/Euclidean modes and pseudo-inertia J for the
% Bregman mode. CFG.gravity follows the paper's compensation convention.
if nargin < 5, dtAdapt = []; end

%% Unpack the current and desired body states

H = state.H; V = state.V(:); Hd = desired.H; Vd = desired.V(:); VdDot = desired.Vdot(:);

%% Configuration error and its left-trivialized covector

He = agc.math.invSE3(Hd) * H;
[eH, psi] = agc.paper.errors('potential', He, cfg.KR, cfg.Kxi);

% Transport the desired body twist from Hd to the current error frame.
transportedVd = agc.math.adjointSE3(agc.math.invSE3(He)) * Vd;
Ve = V - transportedVd;

%% Sliding variable and reference body velocity

Lambda = cfg.Lambda;
Vr = transportedVd - Lambda * eH;

% Differentiate Vr using the error kinematics, not finite differences.
eHdot = agc.paper.errors('potential-derivative', He, Ve, cfg.KR, cfg.Kxi);
VrDot = -agc.math.adTwist(Ve) * transportedVd + agc.math.adjointSE3(agc.math.invSE3(He)) * VdDot ...
    - Lambda * eHdot;

%% Estimated inertial model and transverse dissipation

[piHat, Jhat] = parameterState(cfg.mode, estimate);
I6 = agc.math.inertiaFromPi(piHat);

% This diagnostic is the paper gravity wrench. The regressor below uses
% the same convention to construct the commanded compensation wrench.
gBody = H(1:3,1:3).' * cfg.gravity(:);
Wg = [agc.math.skew(piHat(2:4)) * gBody; piHat(1) * gBody];

s = V - Vr;
LinvS = Lambda \ s;
sNorm = sqrt(max(0, s.' * LinvS));
if sNorm == 0
    D = zeros(6,1);
else
    D = (cfg.kd + cfg.ks * sNorm^(cfg.alpha - 1)) * LinvS;
end

%% Wrench command and optional parameter update

Y = agc.paper.regressor(H, V, Vr, VrDot, cfg.gravity, cfg.coriolis);
W = Y * piHat - D;

% Keep intermediate geometric quantities available to simulation logs and
% theory tests without recomputing the controller pipeline.
diagnostics = struct('He', He, 'Ve', Ve, 'eH', eH, 'eHdot', eHdot, 'Psi', psi, ...
    'Vr', Vr, 'VrDot', VrDot, 's', s, 'D', D, 'Wg', Wg, 'Y', Y, ...
    'Vs', 0.5 * s.' * I6 * s, 'sNorm', sNorm);
estimateNext = estimate;
if isempty(dtAdapt), return; end

% The plant update and parameter update can run at different fixed rates.
switch lower(string(cfg.mode))
    case "euclidean"
        % gammaE stores the ten independent Euclidean adaptation gains.
        estimateNext = agc.paper.adaptation('euclidean-step', piHat, Y.' * s, diag(cfg.gammaE), dtAdapt);
    case "bregman"
        G = pseudoGradient(Y.' * s);
        estimateNext = agc.paper.adaptation('bregman-step', Jhat, G, cfg.gammaB, dtAdapt);
end
end

function [piHat, Jhat] = parameterState(mode, estimate)
%PARAMETERSTATE Convert the mode-specific estimate to inertial parameters.
switch lower(string(mode))
    case {"nominal", "euclidean"}
        piHat = estimate(:); Jhat = [];
    case "bregman"
        Jhat = estimate; piHat = agc.math.piFromPseudo(Jhat);
    otherwise
        error('agc:paper:controller:UnknownMode', 'Unknown controller mode.');
end
end

function G = pseudoGradient(gpi)
%PSEUDOGRADIENT Pull a pi-space gradient back to symmetric pseudo-inertia.
gpi = gpi(:); gi = gpi(5:10);
G = zeros(4);
G(4,4) = gpi(1);
G(1:3,4) = 0.5 * gpi(2:4); G(4,1:3) = G(1:3,4).';
G(1,1) = gi(2) + gi(3); G(2,2) = gi(1) + gi(3); G(3,3) = gi(1) + gi(2);
G(1,2) = -0.5 * gi(4); G(2,1) = G(1,2);
G(1,3) = -0.5 * gi(5); G(3,1) = G(1,3);
G(2,3) = -0.5 * gi(6); G(3,2) = G(2,3);
end
