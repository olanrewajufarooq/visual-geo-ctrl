classdef symInvPotential < fth.ctrl.potential.PotentialBase
    %SYMINVPOTENTIAL Symmetric and invariant potential on SE(3).
    %   Position error: ep = (R^T + Rd^T) xi_e.
    %   Potential function:
    %     Psi(R, xi) = 0.5 tr(K_R (I - Rd^T R)) + 0.5 ep^T K_xi ep
    %
    %   Errors:
    %     eR  = vee(skew(0.5 K_R Re - K_xi ep xi_e^T R))
    %     eXi = (I + R^T Rd) K_xi ep
    %
    %   potType: 'sym-inv'
    properties (Access = private)
        K_R   % 3x3 attitude gain (= Kp_att = diag(Kp(1:3)))
        K_xi  % 3x3 position gain (= Kp_pos = diag(Kp(4:6)))
    end

    methods
        function obj = symInvPotential(K_R, K_xi)
            %SYMINVPOTENTIAL Store attitude and position gains.
            obj.K_R  = K_R;
            obj.K_xi = K_xi;
        end

        function eH = getPotentialError(obj, Hd, H)
            %GETPOTENTIALERROR Compute eH = [eR; eXi].
            st  = fth.se3.poseDecompose(H, Hd);
            ep  = (st.R' + st.Rd') * st.xi_e;
            A   =  -0.5 * obj.K_R * st.Re - obj.K_xi * ep * st.xi_e' * st.R;
            eR  = fth.se3.vee3(fth.se3.skew(A));
            eXi = (eye(3) + st.R' * st.Rd) * obj.K_xi * ep;
            eH  = [eR; eXi];
        end

        function eHDot = getPotentialErrorDerivative(obj, Hd, H, Vd, V)
            %GETPOTENTIALERRORDERIVATIVE Compute eHDot = [eRDot; eXiDot].
            st = fth.se3.trackingState(H, Hd, V, Vd);
            ep = (st.R' + st.Rd') * st.xi_e;

            epDot = -(fth.se3.hat3(st.omega) * st.R' + fth.se3.hat3(st.omega_d) * st.Rd') ...
                      * st.xi_e ...
                    + (st.R' + st.Rd') * (st.R * st.v - st.Rd * st.v_d);

            ADot =  -0.5 * obj.K_R * st.Re * fth.se3.hat3(st.omega_e) ...
                   - obj.K_xi * epDot * st.xi_e' * st.R ...
                   - obj.K_xi * ep * (st.v - st.Re' * st.v_d)' ...
                   - obj.K_xi * ep * st.xi_e' * st.R * fth.se3.hat3(st.omega);

            eRDot  = fth.se3.vee3(fth.se3.skew(ADot));
            eXiDot = -fth.se3.hat3(st.omega_e) * st.Re' * obj.K_xi * ep ...
                     + (eye(3) + st.Re') * obj.K_xi * epDot;
            
            eHDot  = [eRDot; eXiDot];
        end
    end
end
