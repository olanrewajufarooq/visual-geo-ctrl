classdef (Abstract) PotentialBase < handle
    %POTENTIALBASE Interface for attitude/position potentials.
    %   Implementations map pose error into a corrective body wrench.
    %
    %   Output wrench ordering: [torque; force].
    methods (Abstract)
        %COMPUTEWRENCH Return potential wrench from desired/actual pose.
        %   Inputs:
        %     Hd - 4x4 desired pose.
        %     H  - 4x4 actual pose.
        %   Output:
        %     Wp - 6x1 potential wrench.
        Wp = computeWrench(obj, Hd, H)
    end
end
