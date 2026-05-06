classdef NamingUtils
    %NAMINGUTILS Static helpers for compact run/trajectory/controller labels.
    %   Used by SimRunner and BatchRunner for results directory naming.

    methods (Static)
        function label = trajectoryLabel(name)
            %TRAJECTORYLABEL Build a compact trajectory label for folder names.
            switch lower(name)
                case 'circle',       label = 'circle';
                case 'infinity',     label = 'inf';
                case 'infinity3d',   label = 'inf3d';
                case 'infinity3dmod',label = 'inf3dmod';
                case 'lissajous3d',  label = 'liss3d';
                case 'helix3d',      label = 'helix3d';
                case 'poly3d',       label = 'poly3d';
                case 'takeoffland',  label = 'tkoffland';
                otherwise
                    label = regexprep(lower(char(string(name))), '[^a-z0-9]+', '');
            end
            if strlength(string(label)) > 12
                label = char(extractBefore(string(label), 13));
            end
        end

        function label = controllerLabel(cfg)
            %CONTROLLERLABEL Build a compact controller label.
            if ~isfield(cfg.controller, 'type') || isempty(cfg.controller.type)
                label = 'ctrl';
                return;
            end
            switch lower(cfg.controller.type)
                case 'feedforward', label = 'ff';
                case 'feedlin',     label = 'fl';
                case 'pd',          label = 'pd';
                otherwise
                    label = regexprep(lower(char(string(cfg.controller.type))), '[^a-z0-9]+', '');
            end
        end

        function label = potentialLabel(cfg)
            %POTENTIALLABEL Build a compact potential label.
            if ~isfield(cfg.controller, 'potential') || isempty(cfg.controller.potential)
                label = 'pot';
                return;
            end
            switch lower(cfg.controller.potential)
                case 'liealgebra', label = 'lie';
                case 'separate',   label = 'sep';
                otherwise
                    label = regexprep(lower(char(string(cfg.controller.potential))), '[^a-z0-9]+', '');
            end
        end

        function label = batchTrajectoryLabel(cfg)
            %BATCHTRAJECTORYLABEL Return a trajectory label covering the full batch.
            if isfield(cfg.traj, 'batch') && isstruct(cfg.traj.batch) ...
                    && isfield(cfg.traj.batch, 'names') && ~isempty(cfg.traj.batch.names)
                if numel(cfg.traj.batch.names) > 1
                    label = 'multi_traj';
                    return;
                end
                label = vt.sim.NamingUtils.trajectoryLabel(cfg.traj.batch.names{1});
                return;
            end
            label = vt.sim.NamingUtils.trajectoryLabel(cfg.traj.name);
        end

        function label = runLabel(saved, resultsDir)
            %RUNLABEL Return a stable run label for a saved batch child.
            runIndex = [];
            if isfield(saved.cfgSnapshot, 'sim') && isfield(saved.cfgSnapshot.sim, 'batchRunIndex') ...
                    && ~isempty(saved.cfgSnapshot.sim.batchRunIndex)
                runIndex = saved.cfgSnapshot.sim.batchRunIndex;
            else
                [~, savedRunName] = fileparts(resultsDir);
                token = regexp(savedRunName, 'run_(\d+)$', 'tokens', 'once');
                if ~isempty(token)
                    runIndex = str2double(token{1});
                end
            end
            if isempty(runIndex) || ~isfinite(runIndex)
                [~, label] = fileparts(resultsDir);
            else
                label = sprintf('Run %d', runIndex);
            end
        end
    end
end
