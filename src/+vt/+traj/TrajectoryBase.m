classdef (Abstract) TrajectoryBase < handle
    %TRAJECTORYBASE Interface for reference trajectory generators.
    %   Implementations produce desired SE(3) pose, body velocity, and
    %   body acceleration given the current time and state.
    %
    %   Implementations should be deterministic and time-continuous.
    methods (Abstract)
        %GENERATE Return desired pose, velocity, and acceleration.
        %   Inputs:
        %     t - current time [s].
        %     H - 4x4 current pose.
        %     V - 6x1 current body velocity.
        %     params - optional struct for tuning.
        %   Outputs:
        %     Hd - 4x4 desired pose.
        %     Vd - 6x1 desired body velocity.
        %     Ad - 6x1 desired body acceleration.
        [Hd, Vd, Ad] = generate(obj, t, H, V, params)
        %RESET Initialize generator state with a starting pose.
        %   Inputs:
        %     H0 - 4x4 initial pose.
        %     V0 - 6x1 initial body velocity.
        reset(obj, H0, V0)
    end

    methods
        function [H0, V0, A0] = getInitialState(obj)
            %GETINITIALSTATE Convenience method for t=0 desired state.
            %   Outputs:
            %     H0 - 4x4 desired pose at t=0.
            %     V0 - 6x1 desired body velocity at t=0.
            %     A0 - 6x1 desired body acceleration at t=0.
            [H0, V0, A0] = obj.generate(0, eye(4), zeros(6,1), struct());
        end
    end
end
