classdef LieAlgebraPotential < vt.ctrl.potential.PotentialBase
    %LIEALGEBRAPOTENTIAL SE(3) log-map potential.
    %   Uses the log map of the pose error to build a body wrench.
    %
    %   Suitable for coupled attitude/position error shaping.
    properties (Access = private)
        Kp
    end

    methods
        function obj = LieAlgebraPotential(Kp)
            %LIEALGEBRAPOTENTIAL Store gain matrix.
            %   Input:
            %     Kp - 6x6 gain matrix.
            obj.Kp = Kp;
        end

        function Wp = computeWrench(obj, Hd, H)
            %COMPUTEWRENCH Compute wrench using SE(3) error.
            %   Inputs:
            %     Hd - 4x4 desired pose.
            %     H  - 4x4 actual pose.
            %   Output:
            %     Wp - 6x1 potential wrench.
            He = vt.se3.invSE3(Hd) * H;
            zeta = vt.se3.logSE3(He);
            Wp = obj.Kp * zeta;
        end
    end
end
