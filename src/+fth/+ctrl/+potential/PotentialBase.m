classdef (Abstract) PotentialBase < handle
    %POTENTIALBASE Interface for SE(3) potential functions.
    %   Implementations provide the potential error eH and its time
    %   derivative eHDot, used by WrenchController to form the composite
    %   sliding variable s = Ve - lambda*eH.
    %
    %   Output ordering for both methods: [rotational (3x1); translational (3x1)].
    methods (Abstract)
        %GETPOTENTIALERROR Return potential error eH from desired/actual pose.
        %   Inputs:
        %     Hd - 4x4 desired pose (SE(3)).
        %     H  - 4x4 actual pose (SE(3)).
        %   Output:
        %     eH - 6x1 potential error [eR; eXi].
        eH = getPotentialError(obj, Hd, H)

        %GETPOTENTIALERRORDERIVATIVE Return time derivative eHDot.
        %   Inputs:
        %     Hd - 4x4 desired pose (SE(3)).
        %     H  - 4x4 actual pose (SE(3)).
        %     Vd - 6x1 desired body velocity [omega_d; v_d].
        %     V  - 6x1 actual body velocity [omega; v].
        %   Output:
        %     eHDot - 6x1 time derivative of eH.
        eHDot = getPotentialErrorDerivative(obj, Hd, H, Vd, V)
    end
end
