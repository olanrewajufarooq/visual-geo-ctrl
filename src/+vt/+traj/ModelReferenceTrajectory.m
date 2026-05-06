classdef ModelReferenceTrajectory < vt.traj.TrajectoryBase
    %MODELREFERENCETRAJECTORY Filtered reference tracking dynamics.
    %   Tracks a precomputed path using a second-order reference model.
    %
    %   State:
    %     Hd - desired pose.
    %     Vd - desired body velocity.
    properties (Access = private)
        pathGenerator
        lambda
        dt
        Hd
        Vd
        initialized_
    end

    methods
        function obj = ModelReferenceTrajectory(cfg)
            %MODELREFERENCETRAJECTORY Initialize model reference generator.
            %   Input:
            %     cfg - configuration with traj.lambda and sim.dt.
            %   Output:
            %     obj - model reference trajectory generator.
            obj.pathGenerator = vt.traj.PreComputedTrajectory(cfg);
            obj.dt = cfg.sim.dt;
            obj.initialized_ = false;

            if isfield(cfg.traj, 'lambda') && ~isempty(cfg.traj.lambda)
                obj.lambda = cfg.traj.lambda(:);
            else
                obj.lambda = ones(6,1);
            end
        end

        function [Hd, Vd, Ad] = generate(obj, t, H, V, params)
            %GENERATE Return desired state following the reference path.
            %   Inputs:
            %     t - current time [s].
            %     H - current pose (used for initialization).
            %     V - current body velocity (used for initialization).
            %     params - optional struct with lambda override.
            %   Outputs:
            %     Hd - desired pose.
            %     Vd - desired body velocity.
            %     Ad - desired body acceleration.
            if ~obj.initialized_
                obj.initialize(H, V);
            end
            if nargin < 5
                params = struct();
            end

            [H_path, ~, ~] = obj.pathGenerator.generate(t, [], [], []);

            He = vt.se3.invSE3(H_path) * obj.Hd;
            xi = vt.se3.logSE3(He);

            lambda = obj.getLambda(params);
            Vd_dot = -2 * lambda .* obj.Vd - lambda.^2 .* xi;

            obj.Vd = obj.Vd + Vd_dot * obj.dt;
            obj.Hd = obj.Hd * vt.se3.expSE3(vt.se3.hat6(obj.Vd * obj.dt));

            Hd = obj.Hd;
            Vd = obj.Vd;
            Ad = Vd_dot;
        end

        function reset(obj, H0, V0)
            %RESET Set desired pose/velocity and mark initialized.
            %   Inputs:
            %     H0 - desired initial pose.
            %     V0 - desired initial body velocity.
            obj.Hd = H0;
            obj.Vd = V0(:);
            obj.initialized_ = true;
        end
    end

    methods (Access = private)
        function initialize(obj, H, V)
            %INITIALIZE Initialize internal desired state.
            %   Inputs:
            %     H - initial pose.
            %     V - initial body velocity.
            obj.Hd = H;
            obj.Vd = V(:);
            obj.initialized_ = true;
        end

        function lambda = getLambda(obj, params)
            %GETLAMBDA Return lambda vector from params if provided.
            %   Input:
            %     params - struct with optional lambda field.
            %   Output:
            %     lambda - 6x1 gain vector.
            lambda = obj.lambda;
            if nargin < 2 || isempty(params)
                return;
            end
            if isfield(params, 'lambda') && ~isempty(params.lambda)
                lambda = params.lambda(:);
            end
        end
    end
end
