classdef BregmanDivAdaptation < fth.ctrl.adapt.AdaptationBase
%BREGMANDIVADAPTATION Riemannian adaptation on the SPD pseudo-inertia manifold.
% Estimates inertial parameters by evolving a 4×4 symmetric positive
% definite pseudo-inertia matrix J_hat under a natural-gradient law.
%
% Adaptation law:
% G       = vec_inv(E^T * N^T * Y^T * s)   [4×4 gradient direction]
% G_sym   = fth.se3.sym(G)                  [symmetric part]
% J_dot   = -gamma * J_hat * G_sym * J_hat  [Euler step on SPD manifold]
%
% Physical parameters are recovered via:
% pi = N * E * vec(J_hat)  ≡  fth.utils.RBInertia.spd2params(J_hat)
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
        obj.J_hat = fth.utils.RBInertia.params2spd(pi0);

        % Scalar gain: accept a scalar or take first element of a vector
        gamma_raw = cfg.controller.Gamma;
        if isscalar(gamma_raw)
            obj.gamma = gamma_raw;
        else
            obj.gamma = gamma_raw(1);
        end
        assert(obj.gamma > 0, 'BregmanDivAdaptation: gamma must be positive.');

        % Cache constant matrices
        obj.N = fth.utils.RBInertia.spd2params_jacobian();
        obj.E = fth.utils.RBInertia.eliminationspd4();

        obj.updateCount = 0;
        obj.constructBases();
        obj.updateEstimates();
    end

    function params = update(obj, ~, H, ~, V, Ades, dt, s, VR, VRDot)
        %UPDATE Advance pseudo-inertia estimate by one Euler step.
        % Inputs:
        % H     - 4×4 current pose.
        % V     - 6×1 current body velocity.
        % Ades - 6×1 desired acceleration (unused).
        % dt   - timestep [s].
        % s    - 6×1 composite sliding variable Ve + Lambda*eH.
        % VR   - 6×1 reference body velocity.
        % VRDot- 6×1 reference body acceleration.
        % Output:
        % params - struct with updated parameters.
        [~, dt, s, VR, VRDot] = obj.parseUpdateArgs(Ades, dt, s, VR, VRDot);

        Y = obj.regressor(H, V, VR, VRDot);

        % Gradient direction in R^{4×4}: G = vec_inv(E^T * N^T * Y^T * s)
        g_vec = obj.E' * (obj.N' * (Y' * s));   % 16×1
        G     = reshape(g_vec, 4, 4);            % 4×4

        % SPD manifold step: J_dot = -gamma * J * sym(G) * J
        G_sym = fth.se3.sym(G);
        J_dot = -obj.gamma * obj.J_hat * G_sym * obj.J_hat;
        obj.J_hat = obj.J_hat + J_dot * dt;

        obj.updateCount = obj.updateCount + 1;
        obj.updateEstimates();

        params = obj.getParams();
    end

    function params = getParams(obj)
        %GETPARAMS Return current estimated parameters.
        pi_hat = fth.utils.RBInertia.spd2params(obj.J_hat);
        params = struct( ...
            'm', obj.m_hat, ...
            'CoG', obj.cog_hat, ...
            'Iparams', obj.Iparams_hat, ...
            'I6', fth.utils.RBInertia.params2genInertia(pi_hat));
    end

    function diagnostics = getDiagnostics(obj)
        %GETDIAGNOSTICS Return adaptation diagnostics.
        diagnostics = struct( ...
            'J_hat', obj.J_hat, ...
            'is_spd', fth.utils.RBInertia.is_spd(obj.J_hat), ...
            'updateCount', obj.updateCount);
    end

    function setEstimatePi(obj, pi)
        %SETESTIMATEPI Seed estimate from a 10×1 parameter vector.
        % Input:
        % pi - 10×1 vector [m; h; Jparams].
        validateattributes(pi, {'numeric'}, {'vector', 'numel', 10});
        obj.J_hat = fth.utils.RBInertia.params2spd(pi(:));
        obj.updateEstimates();
    end
end

methods (Access = protected)
    function setPayloadEstimate(obj, m_payload, CoG_payload)
        %SETPAYLOADESTIMATE Shift estimates based on payload guess.
        % Overrides base class since Bregman stores J_hat instead of pi_hat.
        % Inputs:
        % m_payload   - payload mass [kg].
        % CoG_payload - 3×1 payload CoG offset [m].
        pi = fth.utils.RBInertia.spd2params(obj.J_hat);
        pi(1) = pi(1) + m_payload;
        pi(2:4) = pi(2:4) + m_payload * CoG_payload(:);
        obj.J_hat = fth.utils.RBInertia.params2spd(pi);
        obj.updateEstimates();
    end
end

methods (Access = private)
    function updateEstimates(obj)
        %UPDATEESTIMATES Unpack J_hat into physical parameter caches.
        pi = fth.utils.RBInertia.spd2params(obj.J_hat);
        [obj.m_hat, obj.cog_hat, obj.Iparams_hat] = obj.unpackPi(pi);
    end
end
end
