classdef TrajPlotter
    %TRAJPLOTTER Static helpers to plot analytic and replay trajectories.
    %   Visualizes reference trajectories (geometry + kinematics) without
    %   running a full simulation.
    %
    %   Typical usage (from a run script):
    %     fth.plot.TrajPlotter.run()
    %     fth.plot.TrajPlotter.run(struct('names', {{'circle','infinity'}}))
    %     fth.plot.TrajPlotter.run(struct('scale', 3, 'duration', 20, ...
    %                                     'goToHoverBeforePathStarts', false))
    %
    %   opts fields (all optional; numeric fields accept a scalar or N-vector
    %   when opts.names lists N trajectories):
    %     .mode                     - 'analytic' (default) or 'replay'
    %     .names                    - analytic name or replay id/label; {} = all
    %     .scale                    - path scale [m]; overrides per-trajectory default
    %     .altitude                 - hover altitude [m]; default 5
    %     .period                   - path cycle duration [s]; preferred over duration
    %     .goToHoverDuration        - hover climb duration [s]; default 30
    %     .goToHoverBeforePathStarts - logical; overrides per-trajectory default
    %     .duration                 - total sim horizon [s]; default 30 (ignored when
    %                                 period is given — duration is then derived)
    %     .dt                       - sampling interval [s]; default 0.02
    %
    %   Lower-level API for custom workflows:
    %     data = fth.plot.TrajPlotter.sampleTrajectory(cfg, dt)
    %     fig  = fth.plot.TrajPlotter.plotSingle(data, cfg)
    %     fig  = fth.plot.TrajPlotter.plotSummary(allData, allCfg)

    methods (Static)
        function run(opts)
            %RUN Sample and plot trajectories, saving PNGs to results/trajectories/.
            %   Input:
            %     opts - (optional) struct with any of these fields:
            %       .mode                     - 'analytic' (default) or 'replay'
            %       .names                    - analytic name or replay id/label; {} = all
            %       .scale                    - path scale [m]; scalar or N-vec
            %       .altitude                 - hover altitude [m]; scalar or N-vec; default 5
            %       .period                   - path cycle duration [s]; scalar or N-vec
            %       .goToHoverDuration        - hover climb duration [s]; scalar or N-vec; default 30
            %       .goToHoverBeforePathStarts - logical; scalar or N-vec
            %       .duration                 - total sim horizon [s]; scalar or N-vec; default 30
            %                                   (ignored per-trajectory when period is given)
            %       .dt                       - sampling interval [s]; scalar or N-vec; default 0.02
            %
            %   Examples:
            %     fth.plot.TrajPlotter.run()
            %     fth.plot.TrajPlotter.run(struct('names', {{'circle','lissajous3d'}}, ...
            %                                     'scale', [3 5], 'period', [40 60]))
            %     fth.plot.TrajPlotter.run(struct('period', 60, 'goToHoverDuration', 30))
            if nargin < 1
                opts = struct();
            end

            mode = lower(char(string(fth.plot.TrajPlotter.getopt(opts, 'mode', 'analytic'))));
            if strcmp(mode, 'replay')
                fth.plot.TrajPlotter.runReplay(opts);
                return;
            elseif ~strcmp(mode, 'analytic')
                error('fth:TrajPlotter:UnknownMode', ...
                    'opts.mode must be ''analytic'' or ''replay''.');
            end

            names                    = fth.plot.TrajPlotter.getopt(opts, 'names',                    {});
            scaleOpt                 = fth.plot.TrajPlotter.getopt(opts, 'scale',                    []);
            altitudeOpt              = fth.plot.TrajPlotter.getopt(opts, 'altitude',                 []);
            periodOpt                = fth.plot.TrajPlotter.getopt(opts, 'period',                   []);
            goToHoverDurationOpt     = fth.plot.TrajPlotter.getopt(opts, 'goToHoverDuration',        []);
            hoverOpt                 = fth.plot.TrajPlotter.getopt(opts, 'goToHoverBeforePathStarts', []);
            durationOpt              = fth.plot.TrajPlotter.getopt(opts, 'duration',                 30);
            dtOpt                    = fth.plot.TrajPlotter.getopt(opts, 'dt',                       0.02);

            specs = fth.plot.TrajPlotter.defaultSpecs();

            if ~isempty(names)
                keep  = cellfun(@(s) any(strcmpi(s, names)), specs);
                specs = specs(keep);
                if isempty(specs)
                    error('fth:TrajPlotter:UnknownName', ...
                        'None of the requested names match available trajectories.');
                end
            end

            outDir = fth.plot.TrajPlotter.resolveOutDir();
            n      = numel(specs);

            % Expand all per-trajectory opts to length n.
            scales            = fth.plot.TrajPlotter.expandOpt(scaleOpt,             n, 'scale');
            altitudes         = fth.plot.TrajPlotter.expandOpt(altitudeOpt,          n, 'altitude');
            periods           = fth.plot.TrajPlotter.expandOpt(periodOpt,            n, 'period');
            goToHoverDurs     = fth.plot.TrajPlotter.expandOpt(goToHoverDurationOpt, n, 'goToHoverDuration');
            hovers            = fth.plot.TrajPlotter.expandOpt(hoverOpt,             n, 'goToHoverBeforePathStarts');
            durations         = fth.plot.TrajPlotter.expandOpt(durationOpt,          n, 'duration');
            dts               = fth.plot.TrajPlotter.expandOpt(dtOpt,               n, 'dt');

            allData = cell(n, 1);
            allCfg  = cell(n, 1);

            for i = 1:n
                hover_i           = hovers(i);
                period_i          = periods(i);
                goToHoverDur_i    = goToHoverDurs(i);
                duration_i        = durations(i);

                % Determine simulation time horizon.
                % If period was given, derive duration; otherwise use duration directly.
                hasPeriod = ~isnan(period_i);
                if hasPeriod
                    if isnan(goToHoverDur_i)
                        goToHoverDur_i = 30;
                    end
                    % hover is unknown until setTrajectory resolves defaults;
                    % use the override if given, else assume true for duration calc.
                    hoverForDur = true;
                    if ~isnan(hover_i)
                        hoverForDur = logical(hover_i);
                    end
                    if hoverForDur
                        dur_i = period_i + goToHoverDur_i;
                    else
                        dur_i = period_i;
                    end
                else
                    dur_i = duration_i;
                end

                cfg = fth.sim.Config();
                cfg.setSimParams(0.005, dur_i);
                if isnan(hover_i)
                    cfg.setTrajectory(specs{i});
                else
                    cfg.setTrajectory(specs{i}, 1, logical(hover_i));
                end

                % Override period after setTrajectory to prevent syncTrajectoryPeriod clobbering.
                if hasPeriod
                    cfg.traj.period      = period_i;
                    cfg.traj.useDuration = false;
                end
                if ~isnan(goToHoverDur_i)
                    cfg.traj.goToHoverDuration = goToHoverDur_i;
                end
                if ~isnan(scales(i))
                    cfg.traj.scale = scales(i);
                end
                if ~isnan(altitudes(i))
                    cfg.traj.altitude = altitudes(i);
                end
                cfg.done();

                dt_i = dts(i);
                if isnan(dt_i), dt_i = 0.02; end

                allCfg{i}  = cfg;
                allData{i} = fth.plot.TrajPlotter.sampleTrajectory(cfg, dt_i);
                fth.plot.TrajPlotter.plotSingle(allData{i}, cfg, outDir);
            end

            fth.plot.TrajPlotter.plotSummary(allData, allCfg, outDir);
        end

        function runReplay(opts)
            %RUNREPLAY Plot processed replay trajectories using the same
            % figures and selection semantics as analytic trajectories.
            names = fth.plot.TrajPlotter.getopt(opts, 'names', {});
            if ischar(names) || isstring(names)
                names = cellstr(names);
            end

            rootDir = ReplayProcessor.defaultRootDir();
            manifestPath = fullfile(rootDir, 'manifest.json');
            manifest = jsondecode(fileread(manifestPath));
            ids = fieldnames(manifest);
            if ~isempty(names)
                keep = false(size(ids));
                for i = 1:numel(ids)
                    keep(i) = any(strcmpi(ids{i}, names)) || ...
                        any(strcmpi(char(string(manifest.(ids{i}).label)), names));
                end
                ids = ids(keep);
                if isempty(ids)
                    error('fth:TrajPlotter:UnknownReplayName', ...
                        'None of the requested replay names match the manifest.');
                end
            end

            outDir = fth.plot.TrajPlotter.resolveOutDir();
            allData = cell(numel(ids), 1);
            allCfg = cell(numel(ids), 1);
            for i = 1:numel(ids)
                entry = manifest.(ids{i});
                artifactPath = fullfile(rootDir, char(string(entry.artifact_file)));
                if ~isfile(artifactPath)
                    error('fth:TrajPlotter:MissingReplayArtifact', ...
                        'Replay artifact is missing: %s', artifactPath);
                end
                loaded = load(artifactPath, 'traj');
                allData{i} = fth.plot.TrajPlotter.replayData(loaded.traj);
                allCfg{i} = struct('traj', struct('name', char(string(entry.label))));
                fprintf('[replay plot %d/%d] %s\n', i, numel(ids), entry.label);
                fth.plot.TrajPlotter.plotSingle(allData{i}, allCfg{i}, outDir);
            end
            fth.plot.TrajPlotter.plotSummary(allData, allCfg, outDir);
            fprintf('[replay plot] Saved figures to %s\n', outDir);
        end

        function data = replayData(traj)
            %REPLAYDATA Adapt a processed replay artifact to plotting fields.
            N = numel(traj.t);
            rpy = zeros(N, 3);
            for k = 1:N
                eul = rotm2eul(traj.R(:, :, k), 'ZYX');
                rpy(k, :) = fliplr(eul);
            end
            data = struct( ...
                't', traj.t(:), ...
                'pos', traj.p, ...
                'rpy', rpy, ...
                'linVel', traj.v_b, ...
                'angVel', traj.omega_b, ...
                'linAcc', traj.a_b, ...
                'angAcc', traj.alpha_b);
        end

        function data = sampleTrajectory(cfg, dt)
            %SAMPLETRAJECTORY Sample a trajectory over its full time horizon.
            %   Inputs:
            %     cfg - fth.sim.Config (after done() has been called).
            %     dt  - sampling interval [s] (e.g. 0.02 for 50 Hz).
            %   Output:
            %     data - struct with fields:
            %       .t        Nx1  time vector [s]
            %       .pos      Nx3  inertial position [x y z] (m)
            %       .rpy      Nx3  orientation [roll pitch yaw] (rad)
            %       .linVel   Nx3  body linear velocity [vx vy vz] (m/s)
            %       .angVel   Nx3  body angular velocity [wx wy wz] (rad/s)
            %       .linAcc   Nx3  body linear acceleration (m/s^2)
            %       .angAcc   Nx3  body angular acceleration (rad/s^2)
            traj = fth.traj.TrajectoryFactory.create(cfg);
            T    = cfg.sim.duration;
            tvec = (0:dt:T).';
            N    = numel(tvec);

            pos    = zeros(N, 3);
            rpy    = zeros(N, 3);
            linVel = zeros(N, 3);
            angVel = zeros(N, 3);
            linAcc = zeros(N, 3);
            angAcc = zeros(N, 3);

            H0 = eye(4);
            V0 = zeros(6, 1);
            for k = 1:N
                [Hd, Vd, Ad] = traj.generate(tvec(k), H0, V0, struct());
                pos(k,:)    = Hd(1:3, 4).';
                eul         = rotm2eul(Hd(1:3, 1:3), 'ZYX');  % [yaw pitch roll]
                rpy(k,:)    = fliplr(eul);                      % [roll pitch yaw]
                angVel(k,:) = Vd(1:3).';
                linVel(k,:) = Vd(4:6).';
                angAcc(k,:) = Ad(1:3).';
                linAcc(k,:) = Ad(4:6).';
            end

            data.t      = tvec;
            data.pos    = pos;
            data.rpy    = rpy;
            data.linVel = linVel;
            data.angVel = angVel;
            data.linAcc = linAcc;
            data.angAcc = angAcc;
        end

        function fig = plotSingle(data, cfg, outDir)
            %PLOTSINGLE Seven-panel detail figure for one trajectory.
            %   Left column: 3D view with start/end markers.
            %   Right two columns: time-series of position, orientation,
            %   linear velocity, angular velocity, linear acceleration,
            %   and angular acceleration.
            %   Inputs:
            %     data   - struct from sampleTrajectory.
            %     cfg    - fth.sim.Config used to generate data.
            %     outDir - (optional) directory to save PNG; omit to skip.
            %   Output:
            %     fig - figure handle.
            if nargin < 3, outDir = ''; end
            name = cfg.traj.name;
            fig  = figure('Name', name, ...
                          'Position', [100 100 1400 800]);
            tl   = tiledlayout(fig, 3, 3, ...
                               'TileSpacing', 'compact', ...
                               'Padding',     'compact');
            title(tl, name, 'FontSize', 14, 'FontWeight', 'bold');

            rgb = fth.plot.TrajPlotter.colorScheme();

            % --- 3D view (col 1, all rows) ---
            ax3d = nexttile(tl, 1, [3 1]);
            fth.plot.TrajPlotter.plot3DPath(ax3d, data);

            % --- Time series (rows 1-3, cols 2-3) ---
            t = data.t;

            axPos = nexttile(tl);
            fth.plot.TrajPlotter.plotTimeSeries(axPos, t, data.pos, rgb, ...
                'Position', 'p (m)', {'x','y','z'});

            axOri = nexttile(tl);
            fth.plot.TrajPlotter.plotTimeSeries(axOri, t, data.rpy, rgb, ...
                'Orientation', '(rad)', {'roll','pitch','yaw'});

            axLV = nexttile(tl);
            fth.plot.TrajPlotter.plotTimeSeries(axLV, t, data.linVel, rgb, ...
                'Linear Velocity', 'v (m/s)', {'vx','vy','vz'});

            axAV = nexttile(tl);
            fth.plot.TrajPlotter.plotTimeSeries(axAV, t, data.angVel, rgb, ...
                'Angular Velocity', '\omega (rad/s)', {'\omegax','\omegay','\omegaz'});

            axLA = nexttile(tl);
            fth.plot.TrajPlotter.plotTimeSeries(axLA, t, data.linAcc, rgb, ...
                'Linear Acceleration', 'a (m/s^2)', {'ax','ay','az'});

            axAA = nexttile(tl);
            fth.plot.TrajPlotter.plotTimeSeries(axAA, t, data.angAcc, rgb, ...
                'Angular Acceleration', '\alpha (rad/s^2)', {'\alphax','\alphay','\alphaz'});

            fth.plot.TrajPlotter.saveFig(fig, outDir, ['traj_' name]);
        end

        function fig = plotSummary(allData, allCfg, outDir)
            %PLOTSUMMARY Summary figure with one 3D view per trajectory.
            %   Inputs:
            %     allData - cell array of data structs (from sampleTrajectory).
            %     allCfg  - cell array of fth.sim.Config objects.
            %     outDir  - (optional) directory to save PNG; omit to skip.
            %   Output:
            %     fig - figure handle.
            if nargin < 3, outDir = ''; end
            n   = numel(allData);
            nC  = min(n, 3);
            nR  = ceil(n / nC);
            fig = figure('Name', 'All Trajectories — 3D View', ...
                         'Position', [50 50 1400 600]);
            tl  = tiledlayout(fig, nR, nC, ...
                              'TileSpacing', 'compact', ...
                              'Padding',     'compact');
            title(tl, 'All Trajectories — 3D View', ...
                  'FontSize', 14, 'FontWeight', 'bold');

            for i = 1:n
                ax = nexttile(tl);
                fth.plot.TrajPlotter.plot3DPath(ax, allData{i});
                title(ax, allCfg{i}.traj.name, 'FontSize', 11);
            end

            fth.plot.TrajPlotter.saveFig(fig, outDir, 'traj_summary');
        end
    end

    % -----------------------------------------------------------------
    methods (Static, Access = private)

        function rgb = colorScheme()
            %COLORSCHEME X=Red Y=Green Z=Blue, matching Plotter.m convention.
            rgb = [1 0 0; 0 1 0; 0 0 1];
        end

        function plot3DPath(ax, data)
            %PLOT3DPATH Draw path with start/end markers on axes ax.
            hold(ax, 'on');
            plot3(ax, data.pos(:,1), data.pos(:,2), data.pos(:,3), ...
                'k-', 'LineWidth', 1.2);
            plot3(ax, data.pos(1,1), data.pos(1,2), data.pos(1,3), ...
                'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', ...
                'DisplayName', 'Start');
            plot3(ax, data.pos(end,1), data.pos(end,2), data.pos(end,3), ...
                'rd', 'MarkerSize', 10, 'MarkerFaceColor', 'r', ...
                'DisplayName', 'End');
            xlabel(ax, 'x (m)');
            ylabel(ax, 'y (m)');
            zlabel(ax, 'z (m)');
            legend(ax, 'Location', 'best', 'FontSize', 10);
            grid(ax, 'on');
            box(ax, 'on');
            axis(ax, 'equal');
            view(ax, 45, 30);
            fth.plot.TrajPlotter.applyPanelStyle(ax);
        end

        function plotTimeSeries(ax, t, data3, rgb, titleStr, yLabel, legLabels)
            %PLOTTIMESERIES Plot three-axis time series on axes ax.
            hold(ax, 'on');
            for i = 1:3
                plot(ax, t, data3(:,i), '-', ...
                    'LineWidth', 1.5, 'Color', rgb(i,:), ...
                    'DisplayName', legLabels{i});
            end
            xlabel(ax, 't (s)');
            ylabel(ax, yLabel);
            title(ax, titleStr);
            legend(ax, 'Location', 'best', 'FontSize', 9);
            fth.plot.TrajPlotter.applyPanelStyle(ax);
        end

        function applyPanelStyle(ax)
            %APPLYPANELSTYLE Shared axis style for all panels.
            grid(ax, 'on');
            box(ax, 'on');
            ax.GridAlpha    = 0.2;
            ax.GridLineStyle = ':';
        end

        function specs = defaultSpecs()
            %DEFAULTSPECS Cell array of trajectory names available for plotting.
            %   Scale and goToHoverBeforePathStarts use per-trajectory defaults
            %   from Config.applyTrajectoryDefinition unless overridden via opts.
            specs = {
                'circle';
                'lissajous3d';
                'helix3d';
                'poly3d';
                'takeoffland';
            };
        end

        function val = getopt(opts, field, default)
            %GETOPT Read a field from an options struct, falling back to default.
            if isfield(opts, field) && ~isempty(opts.(field))
                val = opts.(field);
            else
                val = default;
            end
        end

        function out = expandOpt(val, n, name)
            %EXPANDOPT Expand a scalar opt to an n-element row vector.
            %   Empty → NaN-filled (treated as "not set").
            %   Scalar → repeated n times.
            %   n-element vector → returned as-is.
            %   Other size → error.
            if isempty(val)
                out = nan(1, n);
                return;
            end
            if numel(val) == 1
                out = repmat(double(val), 1, n);
            elseif numel(val) == n
                out = double(val(:).');
            else
                error('fth:TrajPlotter:BadOptSize', ...
                    'opts.%s must be scalar or length %d (got %d).', name, n, numel(val));
            end
        end

        function outDir = resolveOutDir()
            %RESOLVEOUTDIR Create a timestamped results/trajectories/<timestamp>/ directory.
            %   Mirrors the nominal/adaptive pattern from ResultsManager.createResultsDir.
            timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
            base   = fullfile(fth.io.ResultsManager.repoRoot(), 'results', 'trajectories');
            outDir = fullfile(base, timestamp);
            if ~exist(outDir, 'dir')
                mkdir(outDir);
            end
        end

        function saveFig(fig, outDir, filename)
            %SAVEFIG Save figure as PNG if outDir is non-empty.
            if isempty(outDir)
                return;
            end
            if ~exist(outDir, 'dir')
                mkdir(outDir);
            end
            filepath = fullfile(outDir, [filename '.png']);
            set(fig, 'Color', 'w');
            set(fig, 'PaperPositionMode', 'auto');
            drawnow;
            try
                print(fig, filepath, '-dpng', '-r150');
            catch
                try
                    saveas(fig, filepath);
                catch
                    exportgraphics(fig, filepath, 'Resolution', 150);
                end
            end
        end
    end
end
