classdef BregmanDivAdaptation < fth.ctrl.adapt.AdaptationBase
%BREGMANDIVADAPTATION Riemannian adaptation on the SPD pseudo-inertia manifold.
% Estimates inertial parameters by evolving a 4×4 symmetric positive
% definite pseudo-inertia matrix J_hat under a natural-gradient law.
%
% Adaptation law:
% G       = vec_inv(E^T * N^T * Y^T * s)   [4×4 gradient direction]
% G_sym   = fth.se3.symOfMat(G)             [symmetric part]
% J_dot   = -gamma * J_hat * G_sym * J_hat  [Euler step on SPD manifold]
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
end

methods
    function obj = BregmanDivAdaptation(cfg)
        %BREGMANDIVADAPTATION Initialize pseudo-inertia estimate and gain.
        % Input:
        % cfg - configuration with vehicle and controller fields.
        %       cfg.controller.Gamma may be a scalar or 10-vector;
        %       the first element is used as the scalar gain gamma.
        obj.g = cfg.vehicle.g;
        obj.dt = cfg.sim.adaptation_dt;

        % Build initial pi from config parameters
        m = cfg.vehicle.m;
        CoG = cfg.vehicle.CoG(:);
        pi0 = obj.packPi(m, CoG, cfg.vehicle.I_params(:));

        % Convert to pseudo-inertia matrix
        obj.J_hat = fth.ctrl.adapt.AdaptationUtils.params2spd(pi0);

        % Scalar gain: accept a scalar or take first element of a vector
        gamma_raw = cfg.controller.Gamma;
        if isscalar(gamma_raw)
            obj.gamma = gamma_raw;
        else
            obj.gamma = gamma_raw(1);
        end
        assert(obj.gamma > 0, 'BregmanDivAdaptation: gamma must be positive.');

        % Cache constant matrices
        obj.N = fth.ctrl.adapt.AdaptationUtils.spd2params_jacobian();
        obj.E = fth.ctrl.adapt.AdaptationUtils.eliminationspd4();

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
        A     = sqrtm(obj.J_hat);
        B     = A * G_sym * A;
        obj.J_hat       = fth.se3.symOfMat(A * expm(-obj.gamma * dt * B) * A);
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
            'm', obj.m_hat, ...
            'CoG', obj.cog_hat, ...
            'Iparams', obj.Iparams_hat, ...
            'I6', fth.ctrl.adapt.AdaptationUtils.params2genInertia(pi_hat));
    end

    function diagnostics = getDiagnostics(obj)
        %GETDIAGNOSTICS Return adaptation diagnostics.
        diagnostics = struct( ...
            'J_hat', obj.J_hat, ...
            'is_spd', fth.ctrl.adapt.AdaptationUtils.is_spd(obj.J_hat), ...
            'updateCount', obj.updateCount);
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
