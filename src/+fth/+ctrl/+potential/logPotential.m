classdef logPotential < fth.ctrl.potential.PotentialBase
    %LOGPOTENTIAL SE(3) log-map potential.
    %   Uses the log map of the pose error He = Hd^{-1}*H to compute eH.
    %   The error derivative uses the right-trivialized differential of the
    %   SE(3) log map (first-order approximation via dlogSE3).
    %
    %   potType: 'log'
    properties (Access = private)
        Kp  % 6x6 gain matrix (blkdiag of K_R and K_xi)
    end

    methods
        function obj = logPotential(Kp)
            %LOGPOTENTIAL Store gain matrix.
            %   Input:
            %     Kp - 6x6 gain matrix.
            obj.Kp = Kp;
        end

        function eH = getPotentialError(obj, Hd, H)
            %GETPOTENTIALERROR Compute eH = Kp * log(Hd^{-1} * H).
            st = fth.se3.poseDecompose(H, Hd);
            eH = obj.Kp * fth.se3.logSE3(st.He);
        end

        function eHDot = getPotentialErrorDerivative(obj, Hd, H, Vd, V)
            %GETPOTENTIALERRORDERIVATIVE Compute eHDot using dlogSE3.
            %   Body velocity of He approximated as Ve = V - Ad^{-1}(He)*Vd.
            st    = fth.se3.trackingState(H, Hd, V, Vd);
            eHDot = obj.Kp * fth.se3.dlogSE3(st.He, st.Ve);
        end
    end
end
