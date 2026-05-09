classdef (Abstract) CoriolisFactorBase < handle
    %CORIOLISFACTORBASE Interface for Coriolis factorization forms.
    %   A Coriolis factorization computes the 6x6 matrix C such that
    %   the full Coriolis wrench is c = C * VR.
    %
    %   Factorizing rather than returning the wrench directly allows C to
    %   be reused wherever the Coriolis structure appears (e.g., adaptation
    %   regressors, stability analysis).
    methods (Abstract)
        %GETCORIOLISFACTOR Return 6x6 Coriolis factorization matrix.
        %   The full Coriolis wrench is then: coriolis = C * VR.
        %   Inputs:
        %     VR - 6x1 reference velocity.
        %     I6 - 6x6 generalized inertia matrix.
        %   Output:
        %     C  - 6x6 Coriolis factorization matrix.
        C = getCoriolisFactor(obj, VR, I6)
    end
end
