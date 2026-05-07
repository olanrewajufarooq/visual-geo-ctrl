classdef TrajectoryFactory
    %TRAJECTORYFACTORY Build trajectory generators from config.
    %   Returns an AnalyticTraj configured from cfg.traj.
    methods (Static)
        function traj = create(cfg)
            %CREATE Instantiate a trajectory generator.
            %   Input:
            %     cfg - configuration struct with traj fields.
            %   Output:
            %     traj - TrajectoryBase implementation.
            traj = fth.traj.AnalyticTraj(cfg);
        end
    end
end
