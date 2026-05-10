classdef WrenchController < handle
    %WRENCHCONTROLLER Computes body wrench commands from tracking errors.
    %   Implements a composite-variable geometric control law on SE(3):
    %
    %     s     = Ve + Lambda * eH           (sliding variable)
    %     VR    = Ad^{-1}(He) * Vd - Lambda * eH   (reference velocity)
    %     VRDot = -ad(Ve)*Ad^{-1}(He)*Vd + Ad^{-1}(He)*VdDot - Lambda*eHDot
    %     C     = getCoriolisFactor(VR, I6)  (6x6 Coriolis factorization)
    %     W     = I6 * VRDot + C*VR - Wg - Kd * s
    %
    %   where Ve = V - Ad^{-1}(He)*Vd is the classical velocity error,
    %   eH and eHDot come from the configured potential function, and
    %   Lambda is a 6x6 diagonal coupling gain (zero by default).
    %
    %   Output: W - 6x1 commanded body wrench [torque; force].
    properties (Access = private)
        potential
        coriolisFactor
        adaptation
        Kd      % 6x6 diagonal derivative gain
        lambda  % 6x6 diagonal coupling gain (Lambda)
        g
    end

    methods
        function obj = WrenchController(cfg)
            %WRENCHCONTROLLER Configure controller from config.
            %   Input:
            %     cfg - fth.sim.Config instance.
            obj.potential      = fth.ctrl.potential.PotentialFactory.create(cfg);
            obj.coriolisFactor = fth.ctrl.coriolis.CoriolisFactorFactory.create(cfg);
            obj.adaptation     = fth.ctrl.adapt.AdaptationFactory.create(cfg);
            obj.Kd     = diag(cfg.controller.Kd(:));
            obj.g      = cfg.vehicle.g;

            if isfield(cfg.controller, 'lambda') && ~isempty(cfg.controller.lambda)
                lam = cfg.controller.lambda(:);
                if isscalar(lam)
                    obj.lambda = lam * eye(6);
                else
                    obj.lambda = diag(lam);
                end
            else
                obj.lambda = zeros(6);
            end
        end

        function W = computeWrench(obj, Hd, H, Vd, V, Ades, ~) %#ok<INUSD>
            %COMPUTEWRENCH Compute commanded wrench for current state.
            %   Inputs:
            %     Hd   - 4x4 desired pose.
            %     H    - 4x4 current pose.
            %     Vd   - 6x1 desired body velocity.
            %     V    - 6x1 current body velocity.
            %     Ades - 6x1 desired body acceleration (VdDot). Optional.
            %     (7th argument accepted for API compatibility, ignored.)
            %
            %   Output:
            %     W - 6x1 commanded body wrench [torque; force].
            if nargin < 6 || isempty(Ades)
                Ades = zeros(6,1);
            end

            ts = fth.se3.trackingState(H, Hd, V, Vd);

            eH    = obj.potential.getPotentialError(Hd, H);
            eHDot = obj.potential.getPotentialErrorDerivative(Hd, H, Vd, V);

            params = obj.adaptation.getParams();
            I6  = params.I6;
            Wg  = obj.gravityWrench(H, params.m, params.CoG);

            VR    = ts.AdInvHe * Vd - obj.lambda * eH;
            s     = ts.Ve + obj.lambda * eH;
            VRDot = -fth.se3.adV(ts.Ve) * ts.AdInvHe * Vd ...
                    + ts.AdInvHe * Ades ...
                    - obj.lambda * eHDot;

            C        = obj.coriolisFactor.getCoriolisFactor(VR, I6);
            coriolis = C * VR;
            W = I6 * VRDot + coriolis + Wg - obj.Kd * s;
        end

        function [m_hat, cog_hat, Iparams_hat] = getEstimate(obj)
            %GETESTIMATE Return current parameter estimates (if any).
            [m_hat, cog_hat, Iparams_hat] = obj.adaptation.getEstimate();
        end

        function diagnostics = getAdaptationDiagnostics(obj)
            %GETADAPTATIONDIAGNOSTICS Return adaptation diagnostics.
            diagnostics = obj.adaptation.getDiagnostics();
        end

        function setPayloadEstimate(obj, m_payload, CoG_payload)
            %SETPAYLOADESTIMATE Seed estimator with payload values.
            if ismethod(obj.adaptation, 'setPayloadEstimate')
                obj.adaptation.setPayloadEstimate(m_payload, CoG_payload);
            end
        end

        function setEstimatePi(obj, pi)
            %SETESTIMATEPI Seed the adaptive estimate from a pi vector.
            %   Input:
            %     pi - 10×1 parameter vector [m; h; Jparams].
            if ismethod(obj.adaptation, 'setEstimatePi')
                obj.adaptation.setEstimatePi(pi);
            end
        end

        function updateAdaptation(obj, Hd, H, Vd, V, Ades, dt)
            %UPDATEADAPTATION Update adaptation law with latest data.
            %   Computes the composite sliding variable s, reference velocity VR,
            %   and reference acceleration VRDot, then forwards to the adaptation law.
            if nargin < 6 || isempty(Ades); Ades = zeros(6,1); end
            if nargin < 7 || isempty(dt);   dt   = [];         end

            ts    = fth.se3.trackingState(H, Hd, V, Vd);
            eH    = obj.potential.getPotentialError(Hd, H);
            eHDot = obj.potential.getPotentialErrorDerivative(Hd, H, Vd, V);

            VR    = ts.AdInvHe * Vd - obj.lambda * eH;
            VRDot = -fth.se3.adV(ts.Ve) * ts.AdInvHe * Vd ...
                    + ts.AdInvHe * Ades ...
                    - obj.lambda * eHDot;
            s     = ts.Ve + obj.lambda * eH;

            obj.adaptation.update(Hd, H, Vd, V, Ades, dt, s, VR, VRDot);
        end
    end

    methods (Access = private)
        function Wg = gravityWrench(obj, H, m, CoG)
            %GRAVITYWRENCH Compute gravity wrench in body frame.
            R    = H(1:3,1:3);
            gvec = [0; 0; obj.g];
            f_g  = m * (R' * gvec);
            tau_g = cross(CoG(:), f_g);
            Wg   = [tau_g; f_g];
        end
    end
end
