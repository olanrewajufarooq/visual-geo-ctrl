classdef EuclideanAdaptation < fth.ctrl.adapt.AdaptationBase
%EUCLIDEANADAPTATION Gradient adaptation in Euclidean parameter space.
% Estimates mass, first moment, and inertia parameters using a linear regressor.
%
% The 10×1 parameter vector is:
% pi = [m, hx, hy, hz, Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
% where h = m * CoG is the first moment of mass.
%
% Adaptation law:
% pi_hat_dot = Gamma * Y(H, V, VR, VRDot)^T * s
% Gamma may be scalar (expanded to Gamma*I_10) or a 10-vector.
% where s = Ve + Lambda*eH is the composite sliding variable from
% ControllerWrench, and Y is the 6×10 composite reference regressor
% including a gravity contribution.

properties (Access = private)
    pi_hat
    Gamma
    infoMatrix
end

methods
    function obj = EuclideanAdaptation(cfg)
        %EUCLIDEANADAPTATION Initialize estimates and gain matrix.
        % Input:
        % cfg - configuration with vehicle and controller fields.
        obj.g = cfg.vehicle.g;
        obj.dt = cfg.sim.adaptation_dt;

        m = cfg.vehicle.m;
        CoG = cfg.vehicle.CoG(:);
        obj.pi_hat = obj.packPi(m, CoG, cfg.vehicle.I_params(:));

        gamma_raw = cfg.controller.Gamma;
        if isscalar(gamma_raw)
            obj.Gamma = gamma_raw * eye(10);
        else
            obj.Gamma = diag(gamma_raw(:));
        end
        validateattributes(obj.Gamma, {'numeric'}, {'size', [10, 10]}, ...
            'EuclideanAdaptation', 'Gamma');

        obj.infoMatrix  = zeros(10, 10);
        obj.updateCount    = 0;
        obj.spdValidCount   = 0;
        obj.spdInvalidCount = 0;
        obj.updateEstimates();
        obj.initFromCfg(cfg);
    end

    function pi = doUpdate(obj, dt, s, Y)
        %DOUPDATE Step parameter estimates using the pre-computed regressor.
        % Inputs:
        % dt - timestep [s].
        % s  - 6×1 composite sliding variable Ve + Lambda*eH.
        % Y  - 6×10 regressor matrix (pre-computed by base class).
        % Output:
        % pi - 10×1 updated parameter vector.
        obj.infoMatrix  = obj.infoMatrix + dt * (Y.' * Y);
        obj.updateCount = obj.updateCount + 1;
        obj.pi_hat      = obj.pi_hat - obj.Gamma * (Y.' * s) * dt;
        obj.updateEstimates();
        obj.recordSPDStatus(fth.ctrl.adapt.AdaptationUtils.is_valid_params(obj.pi_hat));
        pi = obj.pi_hat;
    end

    function pi = getPi(obj)
        %GETPI Return current parameter estimate.
        pi = obj.pi_hat;
    end

    function params = getParams(obj)
        %GETPARAMS Return current estimated parameters.
        params = struct( ...
            'm', obj.m_hat, ...
            'CoG', obj.cog_hat, ...
            'Iparams', obj.Iparams_hat, ...
            'I6', fth.ctrl.adapt.AdaptationUtils.params2genInertia(obj.pi_hat));
    end

    function diagnostics = getDiagnostics(obj)
        %GETDIAGNOSTICS Return cumulative regressor diagnostics.
        diagnostics = struct( ...
            'infoMatrix',      obj.infoMatrix, ...
            'updateCount',     obj.updateCount, ...
            'spdValidCount',   obj.spdValidCount, ...
            'spdInvalidCount', obj.spdInvalidCount);
    end

    function setEstimatePi(obj, pi)
        %SETESTIMATEPI Replace the adaptive estimate state.
        % Input:
        % pi - 10×1 parameter vector [m; h; Jparams].
        validateattributes(pi, {'numeric'}, {'vector', 'numel', 10});
        obj.pi_hat = pi(:);
        obj.updateEstimates();
    end
end

methods (Access = private)
    function updateEstimates(obj)
        %UPDATEESTIMATES Unpack pi_hat into physical parameter caches.
        [obj.m_hat, obj.cog_hat, obj.Iparams_hat] = obj.unpackPi(obj.pi_hat);
    end
end
end
