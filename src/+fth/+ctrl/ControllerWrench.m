classdef ControllerWrench < handle
    %CONTROLLERWRENCH Computes body wrench commands from tracking errors.
    %   Implements a composite-variable geometric control law on SE(3):
    %
    %     s     = Ve + Lambda * eH           (sliding variable)
    %     VR    = Ad^{-1}(He) * Vd - Lambda * eH   (reference velocity)
    %     VRDot = -ad(Ve)*Ad^{-1}(He)*Vd + Ad^{-1}(He)*VdDot - Lambda*eHDot
    %     pi = [m; m*CoG; Iparams]         (10x1 parameter vector)
    %     W  = Y(H,V,VR,VRDot)*pi - Kd*s  (regressor-based cancellation)
    %
    %   Sliding variable computation is delegated to the adaptation object.
    %   Output: W - 6x1 commanded body wrench [torque; force].
    properties (Access = private)
        adaptation   % AdaptationBase subclass (owns potential, lambda, regressor)
        Kd           % 6×6 derivative gain
        pi_hat       % 10×1 current parameter estimate
    end

    methods
        function obj = ControllerWrench(adaptation, Kd)
            %CONTROLLERWRENCH Create controller with pre-built adaptation and gain.
            %   Inputs:
            %     adaptation - AdaptationBase subclass (owns potential and lambda).
            %     Kd         - 6×6 or 6×1 derivative gain (matrix or vector).
            obj.adaptation = adaptation;
            if isvector(Kd) && numel(Kd) == 6
                obj.Kd = diag(Kd(:));
            else
                obj.Kd = Kd;
            end
            obj.pi_hat = adaptation.getPi();
        end

        function W = computeWrench(obj, Hd, H, Vd, V, Ades, ~)
            %COMPUTEWRENCH Compute commanded wrench for current state.
            %   Inputs:
            %     Hd   - 4x4 desired pose.
            %     H    - 4x4 current pose.
            %     Vd   - 6x1 desired body velocity.
            %     V    - 6x1 current body velocity.
            %     Ades - 6x1 desired body acceleration (optional).
            %     (7th argument accepted for API compatibility, ignored.)
            %   Output:
            %     W - 6x1 commanded body wrench [torque; force].
            if nargin < 6 || isempty(Ades); Ades = zeros(6,1); end
            [s, VR, VRDot] = obj.adaptation.computeSliding(Hd, H, Vd, V, Ades);
            Y = obj.adaptation.regressor(H, V, VR, VRDot);
            W = Y * obj.pi_hat - obj.Kd * s;
        end

        function updatePi(obj, Hd, H, Vd, V, Ades, dt)
            %UPDATEPI Update parameter estimate using current tracking data.
            %   Inputs:
            %     Hd   - 4x4 desired pose.
            %     H    - 4x4 current pose.
            %     Vd   - 6x1 desired body velocity.
            %     V    - 6x1 current body velocity.
            %     Ades - 6x1 desired body acceleration (optional).
            %     dt   - timestep [s] (optional; uses adaptation default if empty).
            if nargin < 6 || isempty(Ades); Ades = zeros(6,1); end
            if nargin < 7;                  dt   = [];         end
            obj.pi_hat = obj.adaptation.update(Hd, H, Vd, V, Ades, dt);
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
    end
end
