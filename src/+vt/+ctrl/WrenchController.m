classdef WrenchController < handle
    %WRENCHCONTROLLER Computes body wrench commands from tracking errors.
    %   Supports PD, feedlinearization, and feedforward modes in SE(3).
    %   Uses a potential function for pose error shaping and (optional)
    %   parameter adaptation for mass/CoG/inertia estimation.
    %
    %   Outputs:
    %     W - 6x1 commanded wrench [torque; force].
    properties (Access = private)
        mode
        potential
        adaptation
        Kd
        g
        control_dt
    end

    methods
        function obj = WrenchController(cfg)
            %WRENCHCONTROLLER Configure controller, potential, and adaptation.
            %   Inputs:
            %     cfg - vt.config.Config instance or equivalent struct.
            %
            %   Output:
            %     obj - WrenchController instance.
            obj.mode = lower(cfg.controller.type);
            obj.potential = vt.ctrl.potential.PotentialFactory.create(cfg);
            obj.adaptation = vt.ctrl.adapt.AdaptationFactory.create(cfg);
            obj.Kd = diag(cfg.controller.Kd(:));
            obj.g = cfg.vehicle.g;
            obj.control_dt = cfg.sim.control_dt;
        end

        function W = computeWrench(obj, Hd, H, Vd, V, Ades, ~)
            %COMPUTEWRENCH Compute commanded wrench for current state.
            %   Inputs:
            %     Hd - 4x4 desired pose.
            %     H  - 4x4 current pose.
            %     Vd - 6x1 desired body velocity.
            %     V  - 6x1 current body velocity.
            %     Ades - 6x1 desired body acceleration (optional).
            %     dt - controller timestep (accepted for API compatibility).
            %
            %   Output:
            %     W - 6x1 commanded body wrench [tau; force].
            if nargin < 6 || isempty(Ades)
                Ades = zeros(6,1);
            end

            He = vt.se3.invSE3(Hd) * H;
            AdInvHe = vt.se3.Ad_inv(He);
            Ve = V - AdInvHe * Vd;

            params = obj.adaptation.getParams();
            I6 = params.I6;
            Wg = obj.gravityWrench(H, params.m, params.CoG);
            Wp = obj.potential.computeWrench(Hd, H);
            Wd = obj.Kd * Ve;

            switch obj.mode
                case 'pd'
                    % Pure PD: potential-based pose error + velocity damping
                    % only.  No model compensation beyond gravity cancellation.
                    W = -Wg - Wp - Wd;
                case 'feedlin'
                    % Feedback linearization: adds Coriolis/centripetal
                    % compensation (C = ad_V^T * I6 * V) to cancel nonlinear
                    % coupling, yielding a linear closed-loop error system.
                    C = vt.se3.adV(V)' * I6 * V;
                    W = -C - Wg - Wp - Wd;
                case 'feedforward'
                    % Full feedforward: Coriolis compensation plus an inertia-
                    % scaled reference acceleration term. Achieves near-perfect
                    % tracking when the model is accurate.
                    C = vt.se3.adV(V)' * I6 * V;
                    ff = I6 * AdInvHe * (Ades + vt.se3.adV(Vd) * (vt.se3.Ad(He) * Ve));
                    W = ff - C - Wg - Wp - Wd;
                otherwise
                    error('Unknown controller type: %s', obj.mode);
            end
        end

        function [m_hat, cog_hat, Iparams_hat] = getEstimate(obj)
            %GETESTIMATE Return current parameter estimates (if any).
            %   Outputs are empty if adaptation is disabled.
            [m_hat, cog_hat, Iparams_hat] = obj.adaptation.getEstimate();
        end

        function diagnostics = getAdaptationDiagnostics(obj)
            %GETADAPTATIONDIAGNOSTICS Return adaptation diagnostics.
            diagnostics = obj.adaptation.getDiagnostics();
        end

        function setPayloadEstimate(obj, m_payload, CoG_payload)
            %SETPAYLOADESTIMATE Seed estimator with payload values.
            %   Inputs:
            %     m_payload - payload mass [kg].
            %     CoG_payload - 3x1 payload CoG offset [m].
            if ismethod(obj.adaptation, 'setPayloadEstimate')
                obj.adaptation.setPayloadEstimate(m_payload, CoG_payload);
            end
        end

        function setEstimateTheta(obj, theta)
            %SETESTIMATETHETA Seed the adaptive estimate from theta.
            %   Input:
            %     theta - 10x1 parameter vector [Iparams; m; m*CoG].
            if ismethod(obj.adaptation, 'setEstimateTheta')
                obj.adaptation.setEstimateTheta(theta);
            end
        end

        function updateAdaptation(obj, Hd, H, Vd, V, Ades, dt)
            %UPDATEADAPTATION Update adaptation law with latest data.
            %   Inputs match computeWrench. Uses dt if provided.
            if nargin < 6 || isempty(Ades)
                Ades = zeros(6,1);
            end
            if nargin < 7 || isempty(dt)
                dt = [];
            end
            obj.adaptation.update(Hd, H, Vd, V, Ades, dt);
        end
    end

    methods (Access = private)
        function Wg = gravityWrench(obj, H, m, CoG)
            %GRAVITYWRENCH Compute gravity wrench in body frame.
            %   Inputs:
            %     H - 4x4 pose.
            %     m - mass [kg].
            %     CoG - 3x1 center of gravity offset [m].
            %
            %   Output:
            %     Wg - 6x1 gravity wrench.
            R = H(1:3,1:3);
            gvec = [0; 0; -obj.g];
            f_g = m * (R' * gvec);
            tau_g = cross(CoG(:), f_g);
            Wg = [tau_g; f_g];
        end
    end
end
