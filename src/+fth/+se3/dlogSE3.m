function dlog = dlogSE3(He, Ve)
%DLOGSE3 Right-trivialized differential of the SE(3) log map.
%   Computes d/dt[log(He)] = J_r^{-1}(log(He)) * Ve using a first-order
%   series approximation of the right-trivialized inverse Jacobian:
%
%       J_r^{-1}(eta) ~= I + 0.5 * ad(eta)
%
%   This approximation is exact at the identity and accurate for small
%   errors. It corresponds to the first two terms of the BCH series.
%
%   Inputs:
%     He - 4x4 pose error in SE(3).
%     Ve - 6x1 body velocity of He [omega_e; v_e].
%
%   Output:
%     dlog - 6x1 time derivative of log(He).

    eta    = fth.se3.logSE3(He);
    ad_eta = fth.se3.adV(eta);
    dlog   = (eye(6) + 0.5 * ad_eta) * Ve;
end
