classdef basicCoriolisFactor < fth.ctrl.coriolis.CoriolisFactorBase
    %BASICCORIOLISFACTOR Standard Euler-Poincare Coriolis factorization.
    %   Computes C = ad(VR)^T * I6.  The full Coriolis wrench is c = C * VR.
    %
    %   This corresponds to the standard body-frame Coriolis/centripetal
    %   term in the Euler-Poincare equations for rigid-body dynamics.
    methods
        function C = getCoriolisFactor(~, VR, I6)
            %GETCORIOLISFACTOR Return ad(VR)^T * I6 (6x6).
            %   Inputs:
            %     VR - 6x1 reference velocity [omega_R; v_R].
            %     I6 - 6x6 generalized inertia matrix.
            %   Output:
            %     C  - 6x6 Coriolis factorization matrix.
            C = -fth.se3.adV(VR)' * I6;
        end
    end
end
