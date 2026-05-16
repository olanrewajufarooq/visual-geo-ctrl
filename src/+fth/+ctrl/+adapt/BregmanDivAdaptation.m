classdef BregmanDivAdaptation < fth.ctrl.adapt.AdaptationBase
%BREGMANDIVADAPTATION Riemannian adaptation on the SPD pseudo-inertia manifold.
% Estimates inertial parameters by evolving a 4×4 symmetric positive
% definite pseudo-inertia matrix J_hat under a natural-gradient law.
%
% Adaptation law:
% G        = vec_inv(E^T * N^T * Y^T * s)       [4×4 gradient direction]
% G_sym    = fth.se3.symOfMat(G)                [symmetric part]
% J_hatDot = -gamma * J_hat * G_sym * J_hat
% J_hat    = J_hat - dt * J_hatDot
%
% Physical parameters are recovered via:
% pi = N * E * vec(J_hat)  ≡  fth.ctrl.adapt.AdaptationUtils.spd2params(J_hat)
%
% Config keyword: 'bregman'

properties (Access = private)
    J_hat       % 4×4 pseudo-inertia estimate (SPD)
    gamma       % scalar positive gain
    N           % 10×10 constant Jacobian (spd2params_jacobian)
    E           % 10×16 constant elimination matrix (eliminationspd4)
    infoMatrix  % 10×10 cumulative regressor information matrix (∫ Y'Y dt)
end

methods
    function obj = BregmanDivAdaptation(cfg)
        %BREGMANDIVADAPTATION Initialize pseudo-inertia estimate and gain.
        % Input:
        % cfg - configuration with vehicle and controller fields.
        %       cfg.controller.Gamma must be a positive scalar gamma.
        obj.g = cfg.vehicle.g;
        obj.dt = cfg.sim.adaptation_dt;

        % Build initial pi from config parameters
        m = cfg.vehicle.m;
        CoG = cfg.vehicle.CoG(:);
        pi0 = obj.packPi(m, CoG, cfg.vehicle.I_params(:));

        % Convert to pseudo-inertia matrix
        obj.J_hat = fth.ctrl.adapt.AdaptationUtils.params2spd(pi0);

        gamma_raw = cfg.controller.Gamma;
        validateattributes(gamma_raw, {'numeric'}, ...
            {'scalar', 'real', 'finite', 'positive'}, ...
            'BregmanDivAdaptation', 'gamma');
        obj.gamma = gamma_raw;

        % Cache constant matrices
        obj.N = fth.ctrl.adapt.AdaptationUtils.spd2params_jacobian();
        obj.E = fth.ctrl.adapt.AdaptationUtils.eliminationspd4();

        obj.infoMatrix = zeros(10, 10);
        obj.updateCount = 0;
        obj.updateEstimates();
        obj.initFromCfg(cfg);
    end

    function pi = doUpdate(obj, dt, s, Y)
        %DOUPDATE Advance pseudo-inertia estimate by one Euler step.
        % Inputs:
        % dt - timestep [s].
        % s  - 6×1 composite sliding variable Ve + Lambda*eH.
        % Y  - 6×10 regressor matrix (pre-computed by base class).
        % Output:
        % pi - 10×1 updated parameter vector.
        g_vec = obj.E' * (obj.N' * (Y' * s));
        G     = reshape(g_vec, 4, 4);
        G_sym = fth.se3.symOfMat(G);
        J_hatDot = -obj.gamma * obj.J_hat * G_sym * obj.J_hat;
        obj.J_hat       = obj.J_hat + dt * J_hatDot;
        obj.infoMatrix  = obj.infoMatrix + dt * (Y.' * Y);
        obj.updateCount = obj.updateCount + 1;
        obj.updateEstimates();
        pi = obj.getPi();
    end

    function pi = getPi(obj)
        %GETPI Return current parameter estimate as a 10×1 vector.
        pi = fth.ctrl.adapt.AdaptationUtils.spd2params(obj.J_hat);
    end

    function params = getParams(obj)
        %GETPARAMS Return current estimated parameters.
        pi_hat = fth.ctrl.adapt.AdaptationUtils.spd2params(obj.J_hat);
        params = struct( ...
            'm',        obj.m_hat, ...
            'CoG',      obj.cog_hat, ...
            'Iparams',  obj.Iparams_hat, ...
            'I6',       fth.ctrl.adapt.AdaptationUtils.params2genInertia(pi_hat));
    end

    function diagnostics = getDiagnostics(obj)
        %GETDIAGNOSTICS Return adaptation diagnostics.
        diagnostics = struct( ...
            'J_hat',        obj.J_hat, ...
            'is_spd',       fth.ctrl.adapt.AdaptationUtils.is_spd(obj.J_hat), ...
            'infoMatrix',   obj.infoMatrix, ...
            'updateCount',  obj.updateCount);
    end

    function setEstimatePi(obj, pi)
        %SETESTIMATEPI Seed estimate from a 10×1 parameter vector.
        % Input:
        % pi - 10×1 vector [m; h; Jparams].
        validateattributes(pi, {'numeric'}, {'vector', 'numel', 10});
        obj.J_hat = fth.ctrl.adapt.AdaptationUtils.params2spd(pi(:));
        obj.updateEstimates();
    end
end

methods (Access = private)
    function updateEstimates(obj)
        %UPDATEESTIMATES Unpack J_hat into physical parameter caches.
        pi = fth.ctrl.adapt.AdaptationUtils.spd2params(obj.J_hat);
        [obj.m_hat, obj.cog_hat, obj.Iparams_hat] = obj.unpackPi(pi);
    end
end
end
