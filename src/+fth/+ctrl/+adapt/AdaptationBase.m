classdef (Abstract) AdaptationBase < handle
    %ADAPTATIONBASE Interface for parameter adaptation laws.
    %   Implementations estimate mass, CoG, and inertia for the controller.
    %
    %   Parameter vector convention (pi, 10×1):
    %     pi = [m, hx, hy, hz, Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
    %   where h = m * CoG is the first moment of mass.
    %
    %   Output struct fields:
    %     m, CoG, Iparams (legacy ordering [Ixx Iyy Izz Ixy Iyz Ixz]), I6.
    methods (Abstract)
        %UPDATE Advance adaptation using tracking data.
        %   Inputs:
        %     Hd    - 4×4 desired pose.
        %     H     - 4×4 current pose.
        %     Vd    - 6×1 desired body velocity.
        %     V     - 6×1 current body velocity.
        %     Ades  - 6×1 desired body acceleration.
        %     dt    - timestep [s].
        %     s     - 6×1 composite sliding variable Ve + Lambda*eH.
        %     VR    - 6×1 reference body velocity.
        %     VRDot - 6×1 reference body acceleration.
        %   Output:
        %     params - struct with updated parameters.
        params = update(obj, Hd, H, Vd, V, Ades, dt, s, VR, VRDot)

        %GETPARAMS Return parameters for the controller/plant.
        %   Output:
        %     params - struct with m, CoG, Iparams, and I6.
        params = getParams(obj)

        %GETESTIMATE Return estimated mass/CoG/inertia parameters.
        %   Outputs:
        %     m_hat       - mass estimate [kg].
        %     cog_hat     - 3×1 CoG estimate [m].
        %     Iparams_hat - 6×1 inertia parameters (legacy ordering).
        [m_hat, cog_hat, Iparams_hat] = getEstimate(obj)

        %GETDIAGNOSTICS Return adaptation diagnostics for offline metrics.
        diagnostics = getDiagnostics(obj)

        %SETESTIMATEPI Seed the adaptive estimate from a 10×1 pi vector.
        %   Input:
        %     pi - 10×1 parameter vector [m; h; Jparams].
        setEstimatePi(obj, pi)
    end
end
