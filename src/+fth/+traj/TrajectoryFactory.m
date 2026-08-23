classdef TrajectoryFactory
    %TRAJECTORYFACTORY Build trajectory generators from config.
    %   Returns a trajectory generator configured from cfg.traj.
    methods (Static)
        function traj = create(cfg)
            %CREATE Instantiate a trajectory generator.
            %   Input:
            %     cfg - configuration struct with traj fields.
            %   Output:
            %     traj - TrajectoryBase implementation.
            if isprop(cfg, 'traj') && isfield(cfg.traj, 'name') ...
                    && strcmpi(string(cfg.traj.name), "replay")
                traj = ReplayTraj(cfg);
                return;
            end
            traj = fth.traj.AnalyticTraj(cfg);
        end
    end
end
