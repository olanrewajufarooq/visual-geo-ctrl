classdef refGainPotential < fth.ctrl.potential.PotentialBase
    %REFGAINPOTENTIAL Potential with gains expressed in the reference frame.
    %   Position error in desired frame: ep = Rd^T xi_e.
    %   Potential function:
    %     Psi(R, xi) = 0.5 tr(K_R (I - Rd^T R)) + 0.5 ep^T K_xi ep
    %
    %   Errors:
    %     eR  = tilde2vec(skewOfMat(0.5 K_R Re - K_xi ep xi_e^T R))
    %     eXi = Re^T K_xi ep
    %
    %   Note: eHDot is not yet derived. Calling getPotentialErrorDerivative
    %   will error until the derivation is complete.
    %
    %   potType: 'ref-gain'
    properties (Access = private)
        K_R   % 3x3 attitude gain (= Kp_att = diag(Kp(1:3)))
        K_xi  % 3x3 position gain (= Kp_pos = diag(Kp(4:6)))
    end

    methods
        function obj = refGainPotential(K_R, K_xi)
            %REFGAINPOTENTIAL Store attitude and position gains.
            obj.K_R  = K_R;
            obj.K_xi = K_xi;
        end

        function eH = getPotentialError(obj, Hd, H)
            %GETPOTENTIALERROR Compute eH = [eR; eXi].
            st  = fth.se3.poseDecompose(H, Hd);
            ep  = st.Rd' * st.xi_e;
            A   =  0.5 * obj.K_R * st.Re - obj.K_xi * ep * st.xi_e' * st.R;
            eR  = fth.se3.tilde2vec(fth.se3.skewOfMat(A));
            eXi = st.Re' * obj.K_xi * ep;
            eH  = [eR; eXi];
        end

        function eHDot = getPotentialErrorDerivative(~, ~, ~, ~, ~)
            %GETPOTENTIALERRORDERIVATIVE Not yet implemented.
            error('fth:refGainPotential:NotImplemented', ...
                'getPotentialErrorDerivative is not yet derived for refGainPotential.');
        end
    end
end
