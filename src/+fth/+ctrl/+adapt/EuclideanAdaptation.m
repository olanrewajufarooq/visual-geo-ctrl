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
% where s = Ve + Lambda*eH is the composite sliding variable from
% WrenchController, and Y is the 6×10 composite reference regressor
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

        obj.Gamma = diag(cfg.controller.Gamma(:));
        validateattributes(obj.Gamma, {'numeric'}, {'size', [10, 10]}, ...
            'EuclideanAdaptation', 'Gamma');

        obj.infoMatrix = zeros(10, 10);
        obj.updateCount = 0;
        obj.updateEstimates();
    end

    function params = update(obj, ~, H, ~, V, Ades, dt, s, VR, VRDot)
        %UPDATE Step parameter estimates using the composite regressor.
        % Inputs:
        % H     - 4×4 current pose.
        % V     - 6×1 current body velocity.
        % Ades - 6×1 desired acceleration (unused; VRDot used instead).
        % dt   - timestep [s].
        % s    - 6×1 composite sliding variable Ve + Lambda*eH.
        % VR   - 6×1 reference body velocity.
        % VRDot- 6×1 reference body acceleration.
        % Output:
        % params - struct with updated parameters.
        [~, dt, s, VR, VRDot] = obj.parseUpdateArgs(Ades, dt, s, VR, VRDot);

        Y = obj.regressor(H, V, VR, VRDot);
        obj.infoMatrix = obj.infoMatrix + dt * (Y.' * Y);
        obj.updateCount = obj.updateCount + 1;
        obj.pi_hat = obj.pi_hat - obj.Gamma * (Y.' * s) * dt;
        obj.updateEstimates();

        params = obj.getParams();
    end

    function params = getParams(obj)
        %GETPARAMS Return current estimated parameters.
        params = struct( ...
            'm', obj.m_hat, ...
            'CoG', obj.cog_hat, ...
            'Iparams', obj.Iparams_hat, ...
            'I6', fth.utils.RBInertia.params2genInertia(obj.pi_hat));
    end

    function diagnostics = getDiagnostics(obj)
        %GETDIAGNOSTICS Return cumulative regressor diagnostics.
        diagnostics = struct( ...
            'infoMatrix', obj.infoMatrix, ...
            'updateCount', obj.updateCount);
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
