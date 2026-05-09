classdef inertiaGainPotential < fth.ctrl.potential.PotentialBase
    %INERTIAGAINPOTENTIAL Potential with gains expressed in the inertia frame.
    %   Position error in inertia frame: eXi = R^T K_xi (xi - xi_d).
    %   Potential function:
    %     Psi(R, xi) = 0.5 tr(K_R (I - Rd^T R)) + 0.5 (xi-xi_d)^T K_xi (xi-xi_d)
    %
    %   Errors:
    %     eR  = vee(skew(0.5 K_R Re))
    %     eXi = R^T K_xi xi_e
    %
    %   potType: 'inertia-gain'
    properties (Access = private)
        K_R   % 3x3 attitude gain (= Kp_att = diag(Kp(1:3)))
        K_xi  % 3x3 position gain (= Kp_pos = diag(Kp(4:6)))
    end

    methods
        function obj = inertiaGainPotential(K_R, K_xi)
            %INERTIAGAINPOTENTIAL Store attitude and position gains.
            obj.K_R  = K_R;
            obj.K_xi = K_xi;
        end

        function eH = getPotentialError(obj, Hd, H)
            %GETPOTENTIALERROR Compute eH = [eR; eXi].
            st  = fth.se3.poseDecompose(H, Hd);
            A   = 0.5 * obj.K_R * st.Re;
            eR  = fth.se3.vee3(fth.se3.skew(A));
            eXi = st.R' * obj.K_xi * st.xi_e;
            eH  = [eR; eXi];
        end

        function eHDot = getPotentialErrorDerivative(obj, Hd, H, Vd, V)
            %GETPOTENTIALERRORDERIVATIVE Compute eHDot = [eRDot; eXiDot].
            st = fth.se3.trackingState(H, Hd, V, Vd);

            A      = -0.5 * obj.K_R * st.Re * fth.se3.hat3(st.omega_e);
            eRDot  = fth.se3.vee3(fth.se3.skew(A));
            eXiDot = -fth.se3.hat3(st.omega) * st.R' * obj.K_xi * st.xi_e ...
                     + st.R' * obj.K_xi * st.R * st.v;
            eHDot  = [eRDot; eXiDot];
        end
    end
end
