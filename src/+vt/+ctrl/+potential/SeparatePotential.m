classdef SeparatePotential < vt.ctrl.potential.PotentialBase
    %SEPARATEPOTENTIAL Split attitude/position potential model.
    %   Uses separate gains for attitude and position error components.
    %
    %   Useful when tuning attitude and position independently.
    properties (Access = private)
        Kp_att
        Kp_pos
    end

    methods
        function obj = SeparatePotential(Kp_att, Kp_pos)
            %SEPARATEPOTENTIAL Store attitude and position gains.
            %   Inputs:
            %     Kp_att - 3x3 attitude gain matrix.
            %     Kp_pos - 3x3 position gain matrix.
            obj.Kp_att = Kp_att;
            obj.Kp_pos = Kp_pos;
        end

        function Wp = computeWrench(obj, Hd, H)
            %COMPUTEWRENCH Compute wrench from attitude/position error.
            %   Inputs:
            %     Hd - 4x4 desired pose.
            %     H  - 4x4 actual pose.
            %   Output:
            %     Wp - 6x1 potential wrench.
            R = H(1:3,1:3);
            p = H(1:3,4);
            Rd = Hd(1:3,1:3);
            pd = Hd(1:3,4);

            R_err = Rd' * R;
            e_p = Rd' * (p - pd);

            F = obj.Kp_pos * R_err' * e_p;
            T = vt.se3.vee3(0.5 * (obj.Kp_att * R_err - (obj.Kp_att * R_err)'));
            Wp = [T; F];
        end
    end
end
