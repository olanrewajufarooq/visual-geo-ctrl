classdef HexacopterPlant < handle
    %HEXACOPTERPLANT Rigid-body hexacopter plant model with ground contact.
    %   Integrates SE(3) dynamics using body wrench inputs and optional
    %   ground contact forces.
    %
    %   State:
    %     H - 4x4 pose (SE(3)).
    %     V - 6x1 body velocity.
    properties
        H
        V
        m
        g
        CoG
        I6
        Iparams
        groundEnable
        groundHeight
        groundStiffness
        groundDamping
        groundFriction
    end

    methods
        function obj = HexacopterPlant(cfg)
            %HEXACOPTERPLANT Initialize mass/inertia and ground settings.
            %   Input:
            %     cfg - config with vehicle and sim fields.
            %
            %   Output:
            %     obj - plant instance with default state.
            obj.m = cfg.vehicle.m;
            obj.g = cfg.vehicle.g;
            obj.CoG = cfg.vehicle.CoG(:);
            obj.Iparams = cfg.vehicle.I_params;
            if isfield(cfg.vehicle,'I6') && ~isempty(cfg.vehicle.I6)
                obj.I6 = cfg.vehicle.I6;
            else
                obj.I6 = fth.utils.getGeneralizedInertia(obj.m, obj.Iparams, obj.CoG);
            end
            obj.H = eye(4);
            obj.V = zeros(6,1);

            obj.groundEnable = false;
            obj.groundHeight = 0;
            obj.groundStiffness = 0;
            obj.groundDamping = 0;
            obj.groundFriction = 0;
            if isprop(cfg, 'sim') && ~isempty(cfg.sim)
                sim = cfg.sim;
                if isfield(sim, 'groundEnable')
                    obj.groundEnable = logical(sim.groundEnable);
                end
                if isfield(sim, 'groundHeight')
                    obj.groundHeight = sim.groundHeight;
                end
                if isfield(sim, 'groundStiffness')
                    obj.groundStiffness = sim.groundStiffness;
                end
                if isfield(sim, 'groundDamping')
                    obj.groundDamping = sim.groundDamping;
                end
                if isfield(sim, 'groundFriction')
                    obj.groundFriction = sim.groundFriction;
                end
            end
        end

        function reset(obj, H0, V0)
            %RESET Set initial pose and body velocity.
            %   Inputs:
            %     H0 - 4x4 pose matrix (optional).
            %     V0 - 6x1 body velocity (optional).
            if nargin < 2 || isempty(H0)
                H0 = eye(4);
            end
            if nargin < 3 || isempty(V0)
                V0 = zeros(6,1);
            end
            obj.H = H0;
            obj.V = V0;
        end

        function [H, V] = getState(obj)
            %GETSTATE Return current pose and body velocity.
            %   Outputs:
            %     H - 4x4 pose matrix.
            %     V - 6x1 body velocity.
            H = obj.H;
            V = obj.V;
        end

        function step(obj, dt, Wprop)
            %STEP Advance dynamics with applied body wrench.
            %   Integrates the Euler-Poincare equation on SE(3):
            %     I6 * Vdot = ad_V^T * I6 * V + W_gravity + W_cmd + W_ground
            %     H_{k+1}  = H_k * exp(hat6(V_mid * dt))  (midpoint rule)
            %
            %   Inputs:
            %     dt - integration step [s].
            %     Wprop - 6x1 body wrench command [torque; force].
            Wg = obj.gravityWrench();
            C = fth.se3.adV(obj.V)' * obj.I6 * obj.V;  % Coriolis/centripetal
            Wground = obj.groundWrench();
            Vdot = obj.I6 \ (C + Wg + Wprop + Wground);

            % Midpoint integration: use V at half-step for the pose update.
            Vmid = obj.V + 0.5 * Vdot * dt;
            obj.H = obj.H * fth.se3.expSE3(fth.se3.hat6(Vmid * dt));
            obj.V = obj.V + Vdot * dt;
        end

        function updateParameters(obj, m, CoG, Iparams)
            %UPDATEPARAMETERS Update mass, CoG, and inertia parameters.
            %   I6 is always recomputed at the end to stay consistent.
            %   Inputs:
            %     m - mass [kg] (optional, pass [] to skip).
            %     CoG - 3x1 center of gravity [m] (optional, pass [] to skip).
            %     Iparams - 1x6 inertia parameters (optional, pass [] to skip).
            if nargin >= 2 && ~isempty(m)
                obj.m = m;
            end
            if nargin >= 3 && ~isempty(CoG)
                obj.CoG = CoG(:);
            end
            if nargin >= 4 && ~isempty(Iparams)
                obj.Iparams = Iparams;
            end
            obj.I6 = fth.utils.getGeneralizedInertia(obj.m, obj.Iparams, obj.CoG);
        end

        function dropPayload(obj, m_new, CoG_new, Iparams_new)
            %DROPPAYLOAD Release payload with momentum conservation.
            %   Adjusts body velocity so that generalized momentum is
            %   preserved across the instantaneous mass/inertia change.
            %
            %   Physics:
            %     p = I6_before * V_before  (momentum before drop)
            %     V_after = I6_after \ p    (solve for new velocity)
            %
            %   Inputs:
            %     m_new - post-drop mass [kg].
            %     CoG_new - 3x1 post-drop center of gravity [m].
            %     Iparams_new - 1x6 post-drop inertia parameters.
            p = obj.I6 * obj.V;
            obj.updateParameters(m_new, CoG_new, Iparams_new);
            obj.V = obj.I6 \ p;
        end
    end

    methods (Access = private)
        function Wg = gravityWrench(obj)
            %GRAVITYWRENCH Compute gravity wrench in body frame.
            %   Output:
            %     Wg - 6x1 wrench due to gravity.
            R = obj.H(1:3,1:3);
            gvec = [0;0;-obj.g];
            f_g = obj.m * (R' * gvec);
            tau_g = cross(obj.CoG, f_g);
            Wg = [tau_g; f_g];
        end

        function Wground = groundWrench(obj)
            %GROUNDWRENCH Compute ground contact wrench if enabled.
            %   Output:
            %     Wground - 6x1 wrench from ground contact.
            Wground = zeros(6,1);
            if ~obj.groundEnable
                return;
            end
            if obj.groundStiffness <= 0
                return;
            end

            p = obj.H(1:3,4);
            depth = obj.groundHeight - p(3);
            if depth <= 0
                return;
            end

            R = obj.H(1:3,1:3);
            v_world = R * obj.V(4:6);
            vz = v_world(3);

            fn = obj.groundStiffness * depth;
            if obj.groundDamping > 0
                fn = fn - obj.groundDamping * min(vz, 0);
            end
            if fn <= 0
                return;
            end

            f_world = [0; 0; fn];
            mu = obj.groundFriction;
            if mu > 0
                v_xy = v_world(1:2);
                speed = norm(v_xy);
                if speed > 1e-6
                    f_tan = mu * fn;
                    f_world(1:2) = -f_tan * (v_xy / speed);
                end
            end

            f_body = R' * f_world;
            Wground = [0; 0; 0; f_body];
        end
    end
end
