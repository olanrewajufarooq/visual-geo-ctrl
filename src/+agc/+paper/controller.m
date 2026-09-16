function [W, diagnostics, estimateNext] = controller(state, desired, cfg, estimate, dtAdapt)
%CONTROLLER Paper-aligned nominal and adaptive body-wrench controller.
% estimate is pi for nominal/Euclidean modes and pseudo-inertia J for Bregman.
if nargin < 5, dtAdapt = []; end
H = state.H; V = state.V(:); Hd = desired.H; Vd = desired.V(:); VdDot = desired.Vdot(:);
He = agc.math.invSE3(Hd) * H;
[eH, psi] = agc.paper.errors('potential', He, cfg.KR, cfg.Kxi);
transportedVd = agc.math.adjointSE3(agc.math.invSE3(He)) * Vd;
Ve = V - transportedVd;
Lambda = cfg.Lambda;
Vr = transportedVd - Lambda * eH;
eHdot = agc.paper.errors('potential-derivative', He, Ve, cfg.KR, cfg.Kxi);
VrDot = -agc.math.adTwist(Ve) * transportedVd + agc.math.adjointSE3(agc.math.invSE3(He)) * VdDot ...
    - Lambda * eHdot;
[piHat, Jhat] = parameterState(cfg.mode, estimate);
I6 = agc.math.inertiaFromPi(piHat);
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
Y = agc.paper.regressor(H, V, Vr, VrDot, cfg.gravity, cfg.coriolis);
W = Y * piHat - D;
diagnostics = struct('He', He, 'Ve', Ve, 'eH', eH, 'eHdot', eHdot, 'Psi', psi, ...
    'Vr', Vr, 'VrDot', VrDot, 's', s, 'D', D, 'Wg', Wg, 'Y', Y, ...
    'Vs', 0.5 * s.' * I6 * s, 'sNorm', sNorm);
estimateNext = estimate;
if isempty(dtAdapt), return; end
switch lower(string(cfg.mode))
    case "euclidean"
        estimateNext = piHat - dtAdapt * cfg.Gamma * (Y.' * s);
    case "bregman"
        G = pseudoGradient(Y.' * s);
        estimateNext = agc.paper.adaptation('bregman-step', Jhat, G, cfg.gammaB, dtAdapt);
end
end

function [piHat, Jhat] = parameterState(mode, estimate)
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
gpi = gpi(:); gi = gpi(5:10);
G = zeros(4);
G(4,4) = gpi(1);
G(1:3,4) = 0.5 * gpi(2:4); G(4,1:3) = G(1:3,4).';
G(1,1) = gi(2) + gi(3); G(2,2) = gi(1) + gi(3); G(3,3) = gi(1) + gi(2);
G(1,2) = -0.5 * gi(4); G(2,1) = G(1,2);
G(1,3) = -0.5 * gi(5); G(3,1) = G(1,3);
G(2,3) = -0.5 * gi(6); G(3,2) = G(2,3);
end
