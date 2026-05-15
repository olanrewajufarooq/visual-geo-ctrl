classdef consistentCoriolisFactor < fth.ctrl.coriolis.CoriolisFactorBase
    %CONSISTENTCORIOLISFACTOR Consistent Coriolis factorization on SE(3).
    %   Computes C = 0.5*(I6*adV(VR) - coadP(I6*VR) - adV(VR)'*I6).
    %   The full Coriolis wrench is c = C * VR.
    %
    %   coriolisFactorization: 'consistent'
    methods
        function C = getCoriolisFactor(~, VR, I6)
            %GETCORIOLISFACTOR Return consistent Coriolis factorization (6x6).
            %   Inputs:
            %     VR - 6x1 reference velocity [omega_R; v_R].
            %     I6 - 6x6 generalized inertia matrix.
            %   Output:
            %     C  - 6x6 Coriolis factorization matrix.
            P = I6 * VR;
            C = 0.5 * (I6 * fth.se3.adV(VR) ...
                       - fth.se3.coadP(P) ...
                       - fth.se3.adV(VR)' * I6);
        end
    end
end
