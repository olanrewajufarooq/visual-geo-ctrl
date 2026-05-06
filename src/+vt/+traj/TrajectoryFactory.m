classdef TrajectoryFactory
    %TRAJECTORYFACTORY Build trajectory generators from config.
    %   Uses cfg.traj.method to select a trajectory implementation.
    %
    %   Supported methods: 'precomputed', 'modelreference'.
    methods (Static)
        function traj = create(cfg)
            %CREATE Instantiate trajectory implementation.
            %   Input:
            %     cfg - configuration struct with traj.method.
            %   Output:
            %     traj - TrajectoryBase implementation.
            method = 'precomputed';
            if isfield(cfg.traj, 'method') && ~isempty(cfg.traj.method)
                method = lower(cfg.traj.method);
            end

            switch method
                case 'precomputed'
                    traj = vt.traj.PreComputedTrajectory(cfg);
                case 'modelreference'
                    traj = vt.traj.ModelReferenceTrajectory(cfg);
                otherwise
                    error('Unknown trajectory method: %s', method);
            end
        end
    end
end
