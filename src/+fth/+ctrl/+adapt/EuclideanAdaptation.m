classdef EuclideanAdaptation < fth.ctrl.adapt.AdaptationBase
    %EUCLIDEANADAPTATION Gradient adaptation in Euclidean parameter space.
    %   Estimates mass, first moment, and inertia parameters using a linear regressor.
    %
    %   The 10×1 parameter vector is:
    %     pi = [m, hx, hy, hz, Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
    %   where h = m * CoG is the first moment of mass.
    %
    %   Adaptation law:
    %     pi_hat_dot = Gamma * Y(H, V, VR, VRDot)^T * s
    %   where s = Ve + Lambda*eH is the composite sliding variable from
    %   WrenchController, and Y is the 6×10 composite reference regressor
    %   including a gravity contribution.
    properties (Access = private)
        g
        pi_hat
        Gamma
        dt
        m_hat
        cog_hat
        Iparams_hat
        infoMatrix
        updateCount
    end

    methods
        function obj = EuclideanAdaptation(cfg)
            %EUCLIDEANADAPTATION Initialize estimates and gain matrix.
            %   Input:
            %     cfg - configuration with vehicle and controller fields.
            obj.g  = cfg.vehicle.g;
            obj.dt = cfg.sim.adaptation_dt;

            m   = cfg.vehicle.m;
            CoG = cfg.vehicle.CoG(:);
            obj.pi_hat = obj.packPi(m, CoG, cfg.vehicle.I_params(:));

            obj.Gamma = diag(cfg.controller.Gamma(:));
            validateattributes(obj.Gamma, {'numeric'}, {'size', [10, 10]}, ...
                'EuclideanAdaptation', 'Gamma');

            obj.infoMatrix  = zeros(10, 10);
            obj.updateCount = 0;
            obj.updateEstimates();
        end

        function params = update(obj, ~, H, ~, V, Ades, dt, s, VR, VRDot)
            %UPDATE Step parameter estimates using the composite regressor.
            %   Inputs:
            %     H     - 4×4 current pose.
            %     V     - 6×1 current body velocity.
            %     Ades  - 6×1 desired acceleration (unused; VRDot used instead).
            %     dt    - timestep [s].
            %     s     - 6×1 composite sliding variable Ve + Lambda*eH.
            %     VR    - 6×1 reference body velocity.
            %     VRDot - 6×1 reference body acceleration.
            %   Output:
            %     params - struct with updated parameters.
            if nargin < 6 || isempty(Ades);  Ades  = zeros(6,1); end %#ok<NASGU>
            if nargin < 7 || isempty(dt);    dt    = obj.dt;     end
            if nargin < 8 || isempty(s);     s     = zeros(6,1); end
            if nargin < 9 || isempty(VR);    VR    = zeros(6,1); end
            if nargin < 10 || isempty(VRDot); VRDot = zeros(6,1); end

            Y = obj.regressor(H, V, VR, VRDot);
            obj.infoMatrix  = obj.infoMatrix + dt * (Y.' * Y);
            obj.updateCount = obj.updateCount + 1;
            obj.pi_hat = obj.pi_hat + obj.Gamma * (Y.' * s) * dt;
            obj.updateEstimates();

            params = obj.getParams();
        end

        function params = getParams(obj)
            %GETPARAMS Return current estimated parameters.
            params = struct( ...
                'm',       obj.m_hat, ...
                'CoG',     obj.cog_hat, ...
                'Iparams', obj.Iparams_hat, ...
                'I6',      fth.utils.RBInertia.params2genInertia(obj.pi_hat));
        end

        function [m_hat, cog_hat, Iparams_hat] = getEstimate(obj)
            %GETESTIMATE Return mass, CoG, and inertia estimates.
            m_hat       = obj.m_hat;
            cog_hat     = obj.cog_hat;
            Iparams_hat = obj.Iparams_hat;
        end

        function diagnostics = getDiagnostics(obj)
            %GETDIAGNOSTICS Return cumulative regressor diagnostics.
            diagnostics = struct( ...
                'infoMatrix',  obj.infoMatrix, ...
                'updateCount', obj.updateCount);
        end

        function setPayloadEstimate(obj, m_payload, CoG_payload)
            %SETPAYLOADESTIMATE Shift estimates based on payload guess.
            %   Inputs:
            %     m_payload   - payload mass [kg].
            %     CoG_payload - 3×1 payload CoG offset [m].
            m_new = obj.pi_hat(1) + m_payload;
            h_new = obj.pi_hat(2:4) + m_payload * CoG_payload(:);
            obj.pi_hat(1)   = m_new;
            obj.pi_hat(2:4) = h_new;
            obj.updateEstimates();
        end

        function setEstimatePi(obj, pi)
            %SETESTIMATEPI Replace the adaptive estimate state.
            %   Input:
            %     pi - 10×1 parameter vector [m; h; Jparams].
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

        function [m_hat, cog_hat, Iparams_hat] = unpackPi(~, pi)
            %UNPACKPI Convert pi vector into physical values.
            %   Input:
            %     pi - 10×1 vector [m; hx; hy; hz; Ixx; Iyy; Izz; Ixy; Ixz; Iyz].
            %   Outputs:
            %     m_hat       - mass estimate.
            %     cog_hat     - 3×1 CoG estimate.
            %     Iparams_hat - 6×1 inertia params in legacy ordering
            %                   [Ixx; Iyy; Izz; Ixy; Iyz; Ixz].
            p = pi(:);
            m_hat   = max(p(1), 1e-9);
            cog_hat = p(2:4) / m_hat;
            % pi(5:10) = [Ixx; Iyy; Izz; Ixy; Ixz; Iyz]
            % Legacy Iparams = [Ixx; Iyy; Izz; Ixy; Iyz; Ixz] (swap 5↔6)
            Iparams_hat = p([5; 6; 7; 8; 10; 9]);
        end

        function pi = packPi(~, m, CoG, Iparams_legacy)
            %PACKPI Build pi from physical parameters.
            %   Iparams_legacy ordering: [Ixx; Iyy; Izz; Ixy; Iyz; Ixz]
            %   pi ordering: [m; hx; hy; hz; Ixx; Iyy; Izz; Ixy; Ixz; Iyz]
            I = Iparams_legacy(:);
            % Reorder: legacy index 5=Iyz→pi(10), legacy index 6=Ixz→pi(9)
            I_pi = [I(1); I(2); I(3); I(4); I(6); I(5)];
            pi = [m; m * CoG(:); I_pi];
        end

        function Y = regressor(obj, H, V, VR, VRDot)
            %REGRESSOR 6×10 composite reference regressor with gravity.
            %   Y(H, V, VR, VRDot) satisfies:
            %     Y * pi = I6*VRDot + ad_V^T*I6*VR + Wg
            %   where Wg is the gravity wrench (linear in m and h).
            %   Inputs:
            %     H     - 4×4 current pose (for gravity rotation).
            %     V     - 6×1 current body velocity.
            %     VR    - 6×1 reference body velocity.
            %     VRDot - 6×1 reference body acceleration.
            %   Output:
            %     Y - 6×10 regressor matrix.
            Y = fth.utils.RBDynamics.paramgenmomentum(VRDot) ...
                - fth.utils.RBDynamics.adjoint(V)' * fth.utils.RBDynamics.paramgenmomentum(VR);

            % Gravity contribution (columns 1..4: m and h)
            R      = H(1:3, 1:3);
            g_body = R' * [0; 0; obj.g];
            Y(4:6, 1)   = Y(4:6, 1)   + g_body;
            Y(1:3, 2:4) = Y(1:3, 2:4) - fth.se3.hat3(g_body);
        end
    end
end
