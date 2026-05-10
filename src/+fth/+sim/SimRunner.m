classdef SimRunner < handle
    %SIMRUNNER Orchestrates simulation, logging, and visualization.
    %   Handles trajectory generation, control updates, plant integration,
    %   logging, and optional live plotting/URDF viewing.
    %
    %   Typical flow:
    %     runner = fth.sim.SimRunner(cfg);
    %     runner.setup();
    %     runner.run('summary', false, false);
    properties
        cfg
        traj
        ctrl
        plant
        log
        plotter
        viewer
        duration
        dt
        control_dt
        adaptation_dt
        N
        resultsDir
        runName
        batchSize
    end
    properties (Access = private)
        figLive
        figFinal
        tCurrent
        kCurrent
        stopped_
        lastLogs
        lastMetrics
        lastEst
        lastRunInfo
        lastIsAdaptive
        massLog
        comLog
        inertiaLog
        estTimeLog
        nextControlTime
        nextAdaptTime
        lastControlTime
        lastAdaptTime
        lastWrench
        payloadMass
        payloadCoG
        payloadDropTime
        batchRunner_
        pendingRunArgs
        console_
        captureConsoleExternally
        executionStartedAt
        executionFinishedAt
        executionWallClockStart
    end
    events
        StepCompleted
        SimulationFinished
    end

    methods
        function obj = SimRunner(cfg)
            %SIMRUNNER Build a runner from a configuration object.
            %   Inputs:
            %     cfg - fth.sim.Config instance (or equivalent struct).
            %
            %   Output:
            %     obj - Simulation runner instance.
            obj.cfg = cfg;
            if ismethod(obj.cfg, 'done')
                obj.cfg.done();
            end
            obj.dt = cfg.sim.dt;
            obj.control_dt = cfg.sim.control_dt;
            obj.adaptation_dt = cfg.sim.adaptation_dt;
            obj.duration = cfg.sim.duration;
            obj.N = floor(obj.duration / obj.dt) + 1;
            obj.stopped_ = false;
            obj.batchSize = obj.resolveBatchSize();
            obj.batchRunner_ = [];
            obj.pendingRunArgs = {};
            obj.captureConsoleExternally = isfield(cfg.sim, 'captureConsoleExternally') ...
                && logical(cfg.sim.captureConsoleExternally);
            obj.console_ = fth.io.ConsoleCapture();
            obj.setupResultsDir();
        end

        function setup(obj)
            %SETUP Initialize trajectory, plant, controller, and logger.
            %   Prepares the plant state and prints key configuration info.
            if obj.isBatchMode()
                fprintf('Configured batch simulation with %d runs.\n', obj.batchSize);
                return;
            end

            if ~obj.captureConsoleExternally
                obj.console_.beginDiary(obj.resultsDir);
            end
            fprintf('%s', fth.io.ConsoleFormatter.section('Setup'));
            fprintf('%s', fth.io.ConsoleFormatter.kv('Trajectory', obj.cfg.traj.name));
            fprintf('%s', fth.io.ConsoleFormatter.kv('Controller', ...
                sprintf('%s (%s)', obj.cfg.controller.type, obj.cfg.controller.potential)));
            if isfield(obj.cfg.controller, 'adaptation') && ~strcmpi(obj.cfg.controller.adaptation, 'none')
                fprintf('%s', fth.io.ConsoleFormatter.kv('Adaptation', obj.cfg.controller.adaptation));
            end
            fprintf('%s', fth.io.ConsoleFormatter.kv('Duration', sprintf('%.1f s', obj.duration)));
            fprintf('\n');
            fprintf('%s', fth.io.ConsoleFormatter.subsection('Timesteps'));
            fprintf('%s', fth.io.ConsoleFormatter.timing(obj.dt, obj.control_dt, obj.adaptation_dt, ...
                isfield(obj.cfg.controller, 'adaptation') && ~strcmpi(obj.cfg.controller.adaptation, 'none')));
            
            % Display gains if available
            if isfield(obj.cfg.controller, 'Kp')
                Kp = obj.cfg.controller.Kp;
                if isvector(Kp) && numel(Kp) == 6
                    fprintf('\n');
                    fprintf('%s', fth.io.ConsoleFormatter.subsection('Gains'));
                    fprintf('%s', fth.io.ConsoleFormatter.vector('Kp', Kp, '%.2f'));
                end
            end
            if isfield(obj.cfg.controller, 'Kd')
                Kd = obj.cfg.controller.Kd;
                if isvector(Kd) && numel(Kd) == 6
                    fprintf('%s', fth.io.ConsoleFormatter.vector('Kd', Kd, '%.2f'));
                end
            end
            if isfield(obj.cfg.controller, 'Gamma') && ~strcmpi(obj.cfg.controller.adaptation, 'none')
                Gamma = obj.cfg.controller.Gamma;
                if isvector(Gamma) && numel(Gamma) == 10
                    fprintf('%s', fth.io.ConsoleFormatter.vector('Adaptive Gains', Gamma, '%.4f'));
                end
            end

            obj.traj = fth.traj.TrajectoryFactory.create(obj.cfg);
            obj.plant = fth.core.Dynamics(obj.cfg);
            obj.ctrl = obj.createController();
            obj.log = fth.core.Logger();
            obj.log.reserve(obj.N);

            [H0, V0] = obj.resolveInitialPlantState();
            obj.plant.reset(H0, V0);
        end

        function setupForRun(obj, isAdaptive, payloadMass, payloadCoG)
            %SETUPFORRUN Apply payload to plant and seed controller parameters.
            %   Called from run() before the simulation loop.
            obj.setupAdaptivePayload(isAdaptive, payloadMass, payloadCoG);
            obj.applyControllerParamInit(isAdaptive, payloadMass, payloadCoG);
        end

        function run(obj, varargin)
            %RUN Execute a nominal or adaptive simulation loop.
            %   Inputs:
            %     Supports the legacy positional run configuration followed
            %     by optional plotType, displayPlots, and saveSimData.
            [isAdaptive, payloadMassArg, payloadCoGArg, payloadDropTimeArg, ...
                plotType, displayPlots, saveSimData] = obj.parseRunInputs(varargin{:});

            if obj.isBatchMode()
                obj.pendingRunArgs = {isAdaptive, payloadMassArg, payloadCoGArg, payloadDropTimeArg, ...
                    plotType, displayPlots, saveSimData};
                obj.runBatch();
                return;
            end

            obj.payloadMass = payloadMassArg;
            obj.payloadCoG = payloadCoGArg(:);
            obj.payloadDropTime = payloadDropTimeArg;

            obj.lastLogs = [];
            obj.lastMetrics = [];
            obj.lastEst = [];
            obj.lastRunInfo = [];
            obj.lastIsAdaptive = isAdaptive;

            obj.setupVisualization();
            obj.setupForRun(isAdaptive, payloadMassArg, payloadCoGArg);

            obj.tCurrent = 0;
            obj.kCurrent = 1;
            obj.stopped_ = false;
            obj.resetEstimationLogs();
            obj.nextControlTime = 0;
            obj.nextAdaptTime = 0;
            obj.lastControlTime = 0;
            obj.lastAdaptTime = 0;
            obj.lastWrench = zeros(6,1);
            obj.executionStartedAt = datetime('now');
            obj.executionFinishedAt = [];
            obj.executionWallClockStart = tic;

            if isAdaptive
                obj.runAdaptiveLoop(payloadDropTimeArg);
            else
                obj.runNominalLoop();
            end

            obj.finalize(isAdaptive);
            obj.persistMetricsFile();
            if string(plotType) ~= "none"
                obj.plotCurrentRun(char(plotType), displayPlots);
            end
            if saveSimData
                obj.persistCurrentRun();
            end
            if ~obj.captureConsoleExternally
                obj.console_.endDiary();
            end
            obj.releaseRunMemory();
        end

        function plot(obj, plotType, displayPlots)
            %PLOT Internal compatibility wrapper for saved-run plotting.
            %   Input:
            %     plotType - 'none','summary','all' (default 'summary').
            %     displayPlots - true to display figures while saving
            %                    (default false).
            if nargin < 2 || isempty(plotType)
                plotType = 'summary';
            end
            if nargin < 3 || isempty(displayPlots)
                displayPlots = false;
            end
            plotType = lower(string(plotType));
            if plotType ~= "none" && plotType ~= "summary" && plotType ~= "all"
                error("plotType must be 'none', 'summary', or 'all'.");
            end
            displayPlots = logical(displayPlots);

            if obj.isBatchMode()
                obj.plotBatch(char(plotType), displayPlots);
                return;
            end

            obj.plotSavedRun(obj.resultsDir, char(plotType), displayPlots);
        end

        function save(obj, varargin)
            %SAVE Deprecated compatibility wrapper for plot().
            plotType = 'summary';
            displayPlots = false;
            if nargin >= 2
                if ischar(varargin{1}) || (isstring(varargin{1}) && isscalar(varargin{1}))
                    plotType = varargin{1};
                    if nargin >= 3
                        displayPlots = varargin{2};
                    end
                elseif nargin >= 3
                    plotType = varargin{2};
                    if nargin >= 4
                        displayPlots = varargin{3};
                    end
                else
                    displayPlots = varargin{1};
                end
            end
            obj.plot(plotType, displayPlots);
        end

        function logs = getLogs(obj)
            %GETLOGS Return the finalized log structure.
            %   Works regardless of whether saveSimData was used.
            %   Output:
            %     logs - struct of time-series arrays.
            if ~isempty(obj.lastLogs)
                logs = obj.lastLogs;
            elseif ~isempty(obj.log)
                logs = obj.log.finalize();
            else
                saved = fth.io.ResultsManager.loadRun(obj.resultsDir);
                logs = saved.logs;
            end
        end

        function stop(obj)
            %STOP Request a graceful simulation stop.
            %   Sets a flag checked by the main loop.
            obj.stopped_ = true;
        end

        function stopped = isStopped(obj)
            %ISSTOPPED Report whether the simulation was stopped.
            %   Output:
            %     stopped - true if stop was requested.
            stopped = obj.stopped_;
        end
    end

    methods (Access = private)
        function [H0, V0] = resolveInitialPlantState(obj)
            %RESOLVEINITIALPLANTSTATE Choose the plant initial condition.
            %   If goToHoverBeforePathStarts is disabled, start from ground origin with
            %   level attitude and zero twist instead of the trajectory's
            %   initial desired state.
            if isprop(obj.cfg, 'traj') && isfield(obj.cfg.traj, 'goToHoverBeforePathStarts') ...
                    && ~logical(obj.cfg.traj.goToHoverBeforePathStarts)
                H0 = eye(4);
                H0(1:3,4) = [0; 0; 0];
                V0 = zeros(6,1);
                return;
            end

            [H0, V0, ~] = obj.traj.getInitialState();
        end

        function ctrl = createController(obj)
            %CREATECONTROLLER Instantiate the configured controller.
            %   Output:
            %     ctrl - fth.ctrl.WrenchController instance.
            ctrl = fth.ctrl.WrenchController(obj.cfg);
        end

        function setupResultsDir(obj)
            %SETUPRESULTSDIR Create a run-specific results folder.
            %   Uses timestamp, trajectory, and controller metadata.
            if isfield(obj.cfg.sim, 'resultsDirOverride') && ~isempty(obj.cfg.sim.resultsDirOverride)
                obj.resultsDir = obj.cfg.sim.resultsDirOverride;
                if ~exist(obj.resultsDir, 'dir')
                    mkdir(obj.resultsDir);
                end
                [~, obj.runName] = fileparts(obj.resultsDir);
                return;
            end

            obj.runName = fth.io.ResultsManager.buildRunName(obj.cfg, obj.isBatchMode());
            obj.resultsDir = fth.io.ResultsManager.createResultsDir( ...
                obj.cfg, fth.io.ResultsManager.repoRoot(), obj.runName);
        end

        function batchCount = resolveBatchSize(obj)
            %RESOLVEBATCHSIZE Resolve requested run count from config gains.
            batchCount = 1;
            if ismethod(obj.cfg, 'getBatchCount')
                batchCount = obj.cfg.getBatchCount();
            end
        end

        function tf = isBatchMode(obj)
            %ISBATCHMODE Return true when multiple runs are configured.
            tf = obj.batchSize > 1;
        end

        function runBatch(obj)
            %RUNBATCH Execute all requested runs via BatchRunner.
            obj.batchRunner_ = fth.sim.BatchRunner(obj.cfg, obj.resultsDir, obj.batchSize);
            obj.batchRunner_.runAll(obj.pendingRunArgs);
            obj.pendingRunArgs = {};
        end

        function plotBatch(obj, plotType, displayPlots)
            %PLOTBATCH Generate plots for each saved batch run.
            if isempty(obj.batchRunner_)
                obj.batchRunner_ = fth.sim.BatchRunner(obj.cfg, obj.resultsDir, obj.batchSize);
            end
            obj.batchRunner_.plotAll(plotType, displayPlots);
        end

        function root = repoRoot(~)
            %REPOROOT Return repository root path.
            root = fth.io.ResultsManager.repoRoot();
        end

        function setupVisualization(obj)
            %SETUPVISUALIZATION Initialize plotter and URDF viewer.
            %   Honors cfg.viz flags and layout preferences.
            obj.plotter = fth.plot.Plotter(obj.resultsDir, struct('savePng', true, 'duration', obj.duration));
            embedUrdf = false;
            if isfield(obj.cfg.viz, 'embedUrdf')
                embedUrdf = logical(obj.cfg.viz.embedUrdf);
            end
            useRobotics = embedUrdf;

            layoutType = 'row-major';
            if isfield(obj.cfg.viz, 'plotLayout') && ~isempty(obj.cfg.viz.plotLayout)
                layoutType = obj.cfg.viz.plotLayout;
            end

            if obj.cfg.viz.enable && obj.cfg.viz.liveSummary
                obj.figLive = figure('Name','Live View','Position',[50 50 1600 900]);
                if obj.lastIsAdaptive
                    if embedUrdf
                        obj.plotter.plotLiveAdaptive(struct('t', []), [], obj.figLive, true, layoutType);
                    end
                else
                    if embedUrdf
                        obj.plotter.plotLiveNominal(struct('t', []), obj.figLive, true, layoutType);
                    end
                end
            else
                obj.figLive = [];
            end

            if obj.cfg.viz.enable
                if embedUrdf && ~isempty(obj.figLive)
                    ax = obj.plotter.getLiveUrdfAxes();
                    obj.viewer = fth.plot.UrdfViewer([], ax, useRobotics);
                    if isgraphics(obj.viewer.ax)
                        title(obj.viewer.ax, 'URDF View');
                    end
                else
                    obj.viewer = fth.plot.UrdfViewer([], [], useRobotics);
                end
                obj.viewer.setAxisLimits(fth.plot.defaultAxisLimits(obj.cfg));
                obj.viewer.setDynamicAxis(obj.cfg.viz.dynamicAxis, obj.cfg.viz.axisPadding);
            end
        end

        function setupAdaptivePayload(obj, isAdaptive, payloadMass, payloadCoG)
            %SETUPADAPTIVEPAYLOAD Apply payload mass to the plant for adaptive runs.
            %   Inputs:
            %     isAdaptive  - true if adaptation is enabled.
            %     payloadMass - payload mass [kg].
            %     payloadCoG  - 3x1 CoG offset [m].
            if ~isAdaptive
                return;
            end

            m_base   = obj.cfg.vehicle.m;
            I_base   = obj.cfg.vehicle.I_params;
            cog_base = obj.cfg.vehicle.CoG(:);
            [m_with, I_with, cog_with] = fth.utils.addPayload(m_base, I_base, cog_base, payloadMass, payloadCoG);

            obj.plant.updateParameters(m_with, cog_with, I_with);
            if payloadMass > 0
                fprintf('%s', fth.io.ConsoleFormatter.kv('Plant mass', sprintf('%.3f kg (with payload)', m_with)));
            else
                fprintf('%s', fth.io.ConsoleFormatter.kv('Plant mass', sprintf('%.3f kg', m_with)));
            end
        end

        function applyControllerParamInit(obj, isAdaptive, payloadMass, payloadCoG)
            %APPLYCONTROLLERPARAMINIT Seed controller with initial parameter estimate.
            %   Resolves cfg.controller.paramInit and calls ctrl.setEstimateTheta.
            %   Works for both nominal and adaptive runs.
            %   For adaptive runs with payload, m_true includes the payload mass.
            %   Inputs:
            %     isAdaptive  - true if adaptation is enabled.
            %     payloadMass - payload mass [kg] (0 for nominal runs).
            %     payloadCoG  - 3x1 CoG offset [m].
            initCfg = obj.getParamInit();
            m_base   = obj.cfg.vehicle.m;
            I_base   = obj.cfg.vehicle.I_params;
            cog_base = obj.cfg.vehicle.CoG(:);
            if isAdaptive && payloadMass > 0
                [m_true, I_true, cog_true] = fth.utils.addPayload(m_base, I_base, cog_base, payloadMass, payloadCoG);
            else
                m_true   = m_base;
                I_true   = I_base;
                cog_true = cog_base;
            end
            [theta0, initLabel] = obj.resolveEstimateInitializationTheta( ...
                initCfg, m_base, I_base, cog_base, m_true, I_true, cog_true);
            obj.ctrl.setEstimateTheta(theta0);
            fprintf('%s', fth.io.ConsoleFormatter.kv('Param init', ...
                sprintf('%s (mass=%.3f kg)', initLabel, theta0(7))));
        end

        function runNominalLoop(obj)
            %RUNNOMINALLOOP Main loop for nominal control.
            for k = 1:obj.N
                if obj.stopped_; break; end
                obj.simulationStep(k, struct());
                obj.tCurrent = obj.tCurrent + obj.dt;
                obj.kCurrent = k;
            end
        end

        function runAdaptiveLoop(obj, dropTime)
            %RUNADAPTIVELOOP Main loop for adaptive control.
            %   Handles payload drop timing and updates estimates.
            dropped = false;
            m_base     = obj.cfg.vehicle.m;
            I_base     = obj.cfg.vehicle.I_params;
            cog_base   = obj.cfg.vehicle.CoG(:);
            for k = 1:obj.N
                if obj.stopped_; break; end
                if ~dropped && obj.tCurrent >= dropTime
                    obj.plant.dropPayload(m_base, cog_base, I_base);
                    dropped = true;
                end
                obj.simulationStep(k, obj.getAdaptiveParams());
                obj.tCurrent = obj.tCurrent + obj.dt;
                obj.kCurrent = k;
            end
        end

        function resetEstimationLogs(obj)
            %RESETESTIMATIONLOGS Clear adaptation history buffers.
            %   Resets mass/CoG/inertia time series.
            obj.massLog = [];
            obj.comLog = [];
            obj.inertiaLog = [];
            obj.estTimeLog = [];
        end

        function simulationStep(obj, k, tparams)
            %SIMULATIONSTEP Single-step update.
            %   Inputs:
            %     k       - step index.
            %     tparams - struct of trajectory params (empty struct for nominal).
            [H, V] = obj.plant.getState();
            [Hd, Vd, Ad] = obj.traj.generate(obj.tCurrent, H, V, tparams);
            obj.maybeUpdateAdaptation(Hd, H, Vd, V, Ad);
            W_cmd = obj.computeControl(Hd, H, Vd, V, Ad);
            obj.logStep(Hd, Vd, Ad, W_cmd, k);
            obj.recordEstimate();
        end

        function W_cmd = computeControl(obj, Hd, H, Vd, V, Ad)
            %COMPUTECONTROL Compute and apply wrench command.
            %   Inputs: desired/actual pose/velocity/acceleration.
            %   Output: W_cmd - 6x1 body wrench applied to plant.
            W_cmd = obj.maybeUpdateControl(Hd, H, Vd, V, Ad);
            obj.plant.step(obj.dt, W_cmd);
        end

        function maybeUpdateAdaptation(obj, Hd, H, Vd, V, Ad)
            %MAYBEUPDATEADAPTATION Update adaptation on schedule.
            %   Uses adaptation_dt to rate-limit updates.
            if obj.shouldUpdate(obj.nextAdaptTime)
                obj.ctrl.updateAdaptation(Hd, H, Vd, V, Ad, obj.adaptation_dt);
                obj.lastAdaptTime = obj.tCurrent;
                obj.nextAdaptTime = obj.nextAdaptTime + obj.adaptation_dt;
            end
        end

        function W_cmd = maybeUpdateControl(obj, Hd, H, Vd, V, Ad)
            %MAYBEUPDATECONTROL Update controller on schedule.
            %   Output: W_cmd - last or updated wrench command.
            if obj.shouldUpdate(obj.nextControlTime) || isempty(obj.lastWrench)
                obj.lastWrench = obj.ctrl.computeWrench(Hd, H, Vd, V, Ad, obj.control_dt);
                obj.lastControlTime = obj.tCurrent;
                obj.nextControlTime = obj.nextControlTime + obj.control_dt;
            end
            W_cmd = obj.lastWrench;
        end

        function doUpdate = shouldUpdate(obj, nextTime)
            %SHOULDUPDATE Check timing guard for updates.
            %   Input: nextTime - scheduled update time.
            %   Output: doUpdate - true when tCurrent passes nextTime.
            tol = max(1e-12, obj.dt * 1e-9);
            doUpdate = obj.tCurrent + tol >= nextTime;
        end

        function actual = buildActual(~, H, V)
            %BUILDACTUAL Package actual state for logging.
            %   Inputs: H (pose), V (body velocity).
            %   Output: actual struct (pos, rpy, linVel, angVel).
            pos = H(1:3,4);
            rpy = fth.utils.rotm2rpy(H(1:3,1:3)).';
            actual = struct('pos', pos', 'rpy', rpy, 'linVel', V(4:6)', 'angVel', V(1:3)');
        end

        function desired = buildDesired(~, Hd, Vd, Ad)
            %BUILDDESIRED Package desired state for logging.
            %   Inputs: Hd, Vd, Ad desired pose/velocity/acceleration.
            %   Output: desired struct for logger.
            pos = Hd(1:3,4);
            rpy = fth.utils.rotm2rpy(Hd(1:3,1:3)).';
            Ad = Ad(:);
            desired = struct('pos', pos', 'rpy', rpy, 'linVel', Vd(4:6)', 'angVel', Vd(1:3)', 'acc6', Ad.');
        end

        function cmd = buildCmd(~, W_cmd)
            %BUILDCMD Package command wrench for logging.
            %   Input: W_cmd 6x1 wrench.
            %   Output: cmd struct with force/torque components.
            cmd = struct('wrenchF', W_cmd(4:6)', 'wrenchT', W_cmd(1:3)');
        end

        function logStep(obj, Hd, Vd, Ad, W_cmd, k)
            %LOGSTEP Append logs and update visualization/safety.
            %   Inputs: desired state, command, and step index.
            [Hn, Vn] = obj.plant.getState();
            actual = obj.buildActual(Hn, Vn);
            desired = obj.buildDesired(Hd, Vd, Ad);
            cmd = obj.buildCmd(W_cmd);
            timing = struct('controlTime', obj.lastControlTime, 'adaptationTime', obj.lastAdaptTime);
            obj.log.append(obj.tCurrent, actual, desired, cmd, timing);

            pos_actual = Hn(1:3,4);
            pos_des = Hd(1:3,4);
            obj.updateVisualization(pos_des, pos_actual, Hn, k);
            obj.checkSafety(pos_actual);
        end

        function recordEstimate(obj)
            %RECORDESTIMATE Save latest parameter estimates.
            %   Stores mass, CoG, and inertia time series if available.
            [m_hat, cog_hat, Iparams_hat] = obj.ctrl.getEstimate();
            if isempty(m_hat)
                return;
            end
            obj.estTimeLog(end+1) = obj.tCurrent;
            obj.massLog(end+1) = m_hat;
            obj.comLog(end+1,:) = cog_hat(:).';
            obj.inertiaLog(end+1,:) = Iparams_hat(:).';
        end

        function params = getAdaptiveParams(obj)
            %GETADAPTIVEPARAMS Build parameter struct from estimates.
            %   Output: params struct with m, cog, and Iparams.
            [m_hat, cog_hat, Iparams_hat] = obj.ctrl.getEstimate();
            params = struct('m', m_hat, 'cog', cog_hat, 'Iparams', Iparams_hat);
        end

        function [theta0, initLabel] = resolveEstimateInitializationTheta( ...
                obj, initCfg, m_base, I_base, cog_base, m_true, I_true, cog_true)
            %RESOLVEESTIMATEINITIALIZATIONTHETA Build initial adaptive theta.
            mode = 'vehicle';
            spec = [];
            if isstruct(initCfg)
                if isfield(initCfg, 'mode') && ~isempty(initCfg.mode)
                    mode = char(lower(string(initCfg.mode)));
                end
                if isfield(initCfg, 'spec')
                    spec = initCfg.spec;
                end
            elseif ischar(initCfg) || (isstring(initCfg) && isscalar(initCfg))
                mode = char(lower(string(initCfg)));
            end

            payloadModes = {'vehicle-plus-payload', 'mid-vehicle-payload', 'vehicle-plus-payload-higher'};
            if any(strcmp(mode, payloadModes)) && m_true <= m_base + eps(m_base)
                error('SimRunner:NoPayloadForInitMode', ...
                    'Initialization mode ''%s'' requires a payload to be configured.', mode);
            end

            switch mode
                case 'vehicle'
                    theta0 = obj.packEstimateTheta(m_base, I_base, cog_base);
                    initLabel = 'VEHICLE';
                case 'vehicle-plus-payload'
                    theta0 = obj.packEstimateTheta(m_true, I_true, cog_true);
                    initLabel = 'VEHICLE-PLUS-PAYLOAD';
                case 'mid-vehicle-payload'
                    if isempty(spec)
                        theta0 = obj.buildDefaultFixedEstimateTheta(m_base, I_base, cog_base, m_true, I_true, cog_true);
                    else
                        validateattributes(spec, {'numeric'}, {'vector', 'numel', 10});
                        theta0 = spec(:);
                    end
                    initLabel = 'MID-VEHICLE-PAYLOAD';
                case 'vehicle-plus-payload-higher'
                    if isempty(spec)
                        theta0 = obj.buildDefaultFixedHigherEstimateTheta(m_base, I_base, cog_base, m_true, I_true, cog_true);
                    else
                        validateattributes(spec, {'numeric'}, {'vector', 'numel', 10});
                        theta0 = spec(:);
                    end
                    initLabel = 'VEHICLE-PLUS-PAYLOAD-HIGHER';
                case 'vehicle-slight-dev'
                    theta0 = obj.buildVehicleSlightDevEstimateTheta(m_base, I_base, cog_base, spec);
                    initLabel = 'VEHICLE-SLIGHT-DEV';
                case 'custom'
                    validateattributes(spec, {'numeric'}, {'vector', 'numel', 10});
                    theta0 = spec(:);
                    initLabel = 'CUSTOM';
                case 'random'
                    theta0 = obj.buildRandomEstimateTheta(m_true, I_true, cog_true, spec);
                    initLabel = 'RANDOM';
                otherwise
                    error('SimRunner:InvalidEstimateInitializationMode', ...
                        'Unknown estimate initialization mode: %s', mode);
            end
        end

        function theta = buildRandomEstimateTheta(obj, m_true, I_true, cog_true, spec)
            %BUILDRANDOMESTIMATETHETA Build a deterministic random initial theta.
            seed = 1729;
            if nargin >= 5 && ~isempty(spec)
                if isnumeric(spec) && isscalar(spec)
                    seed = double(spec);
                elseif isstruct(spec) && isfield(spec, 'seed') && ~isempty(spec.seed)
                    seed = double(spec.seed);
                else
                    error('SimRunner:InvalidRandomEstimateSpec', ...
                        'Random estimate spec must be empty, a scalar seed, or a struct with a seed field.');
                end
            end

            rngState = rng;
            cleanup = onCleanup(@() rng(rngState));
            cleanupObj = cleanup; %#ok<NASGU>
            rng(seed, 'twister');

            inertiaScale = 1 + 0.10 * (2 * rand(6,1) - 1);
            massScale = 1 + 0.10 * (2 * rand() - 1);
            cogDelta = 0.01 * (2 * rand(3,1) - 1);

            I_rand = I_true(:) .* inertiaScale;
            m_rand = max(1e-6, m_true * massScale);
            cog_rand = cog_true(:) + cogDelta;
            theta = obj.packEstimateTheta(m_rand, I_rand, cog_rand);
        end

        function theta = buildDefaultFixedEstimateTheta(obj, m_nom, I_nom, cog_nom, m_true, I_true, cog_true)
            %BUILDDEFAULTFIXEDESTIMATETHETA Build the repo default fixed theta.
            alpha = [0.40; 0.60; 0.45; 0.55; 0.50; 0.42; 0.58; 0.47; 0.53; 0.50];
            theta_nom = obj.packEstimateTheta(m_nom, I_nom, cog_nom);
            theta_true = obj.packEstimateTheta(m_true, I_true, cog_true);
            theta = theta_nom + alpha .* (theta_true - theta_nom);
        end

        function theta = buildDefaultFixedHigherEstimateTheta(obj, m_nom, I_nom, cog_nom, m_true, I_true, cog_true)
            %BUILDDEFAULTFIXEDHIGHERESTIMATETHETA Build the repo default
            % fixed-higher theta above the true loaded value.
            alpha = [0.15; 0.35; 0.20; 0.30; 0.25; 0.18; 0.32; 0.22; 0.28; 0.25];
            theta_nom = obj.packEstimateTheta(m_nom, I_nom, cog_nom);
            theta_true = obj.packEstimateTheta(m_true, I_true, cog_true);
            theta = theta_true + alpha .* (theta_true - theta_nom);
        end

        function theta = buildVehicleSlightDevEstimateTheta(obj, m_base, I_base, cog_base, spec)
            %BUILDVEHICLESLIGHTDEVESTIMATETHETA Build a slightly perturbed vehicle theta.
            %   Default: +5% mass, +5% diagonal inertia terms, +0.01 m CoG z-axis.
            %   If spec is a 10x1 numeric vector it is used directly.
            if nargin >= 5 && ~isempty(spec)
                validateattributes(spec, {'numeric'}, {'vector', 'numel', 10});
                theta = spec(:);
                return;
            end
            m_dev   = m_base * 1.05;
            I_dev   = I_base(:) .* [1.05; 1.04; 1.05; 1.075; 1.05; 1.025];
            cog_dev = cog_base(:) + [0.05; 0.025; 0.075];
            theta   = obj.packEstimateTheta(m_dev, I_dev, cog_dev);
        end

        function theta = packEstimateTheta(~, m, Iparams, CoG)
            %PACKESTIMATETHETA Convert physical parameters into theta form.
            theta = [Iparams(:); m; m * CoG(:)];
        end

        function value = getPayloadField(obj, fieldName, defaultValue)
            %GETPAYLOADFIELD Read payload config with fallback.
            %   Inputs: fieldName, defaultValue.
            %   Output: value from cfg.payload or default.
            value = defaultValue;
            if isprop(obj.cfg, 'payload') && isfield(obj.cfg.payload, fieldName)
                candidate = obj.cfg.payload.(fieldName);
                if ~isempty(candidate)
                    value = candidate;
                end
            end
        end

        function updateVisualization(obj, pos_des, pos_actual, Hn, k)
            %UPDATEVISUALIZATION Render live plots/URDF if enabled.
            %   Inputs: desired/actual position, pose, and step index.
            updateEvery = 1;
            if isfield(obj.cfg.viz, 'updateEvery') && ~isempty(obj.cfg.viz.updateEvery)
                updateEvery = max(1, obj.cfg.viz.updateEvery);
            end
            doUpdate = obj.cfg.viz.enable && mod(k, updateEvery) == 0;

            if doUpdate && ~isempty(obj.viewer) && isgraphics(obj.viewer.ax)
                obj.viewer.showPose(Hn);
                obj.viewer.updatePaths(pos_des, pos_actual);
            end

            if doUpdate && obj.cfg.viz.liveSummary
                logsNow = obj.log.finalize();
                embedUrdf = isfield(obj.cfg.viz, 'embedUrdf') && obj.cfg.viz.embedUrdf;
                layoutType = 'row-major';
                if isfield(obj.cfg.viz, 'plotLayout') && ~isempty(obj.cfg.viz.plotLayout)
                    layoutType = obj.cfg.viz.plotLayout;
                end
                if obj.lastIsAdaptive
                    estNow = obj.getEstimationData(logsNow);
                    obj.figLive = obj.plotter.plotLiveAdaptive(logsNow, estNow, obj.figLive, embedUrdf, layoutType);
                else
                    obj.figLive = obj.plotter.plotLiveNominal(logsNow, obj.figLive, embedUrdf, layoutType);
                end
            end

            if doUpdate && (~isempty(obj.figLive) && isvalid(obj.figLive))
                drawnow limitrate;
            end
        end

        function checkSafety(obj, pos_actual)
            %CHECKSAFETY Enforce numeric and ground-contact checks.
            %   Stops the sim if values are invalid or below ground.
            if any(~isfinite(pos_actual))
                warning('Numerical issue detected. Stopping simulation.');
                obj.stopped_ = true;
                return;
            end

            enableSafety = true;
            if isfield(obj.cfg.sim, 'enableSafety')
                enableSafety = logical(obj.cfg.sim.enableSafety);
            end
            if ~enableSafety
                return;
            end

            minZ = -0.005;
            if isfield(obj.cfg.sim, 'minZ') && ~isempty(obj.cfg.sim.minZ)
                minZ = obj.cfg.sim.minZ;
            end

            if obj.tCurrent > 1.0 && pos_actual(3) < minZ
                warning('Ground contact detected. Stopping simulation.');
                obj.stopped_ = true;
            end
        end

        function finalize(obj, isAdaptive)
            %FINALIZE Compute metrics and cache run artifacts.
            %   Inputs: isAdaptive - whether adaptation was used.
            logs = obj.log.finalize();
            logs = fth.utils.cleanNearZero(logs);
            obj.executionFinishedAt = datetime('now');
            elapsedWallSeconds = toc(obj.executionWallClockStart);
            fprintf('%s', fth.io.ConsoleFormatter.section('Execution'));
            fprintf('%s', fth.io.ConsoleFormatter.kv('Started', char(datetime(obj.executionStartedAt, 'Format', 'yyyy-MM-dd HH:mm:ss'))));
            fprintf('%s', fth.io.ConsoleFormatter.kv('Finished', char(datetime(obj.executionFinishedAt, 'Format', 'yyyy-MM-dd HH:mm:ss'))));
            fprintf('%s', fth.io.ConsoleFormatter.kv('Elapsed', sprintf('%.3f s', elapsedWallSeconds)));
            fprintf('%s\n', fth.io.ConsoleFormatter.note('Simulation completed.'));

            est = [];
            if isAdaptive
                est = obj.getEstimationData(logs);
                logs.est = est;
            end

            metricsObj = fth.core.TrackingMetrics(logs, obj.cfg.traj.name);
            metrics = metricsObj.computeAll();
            metricsObj.printReport();
            fprintf('%s', fth.io.ConsoleFormatter.headline(metrics, isAdaptive));

            obj.lastLogs = logs;
            obj.lastMetrics = metrics;
            obj.lastIsAdaptive = isAdaptive;
            obj.lastEst = est;

            obj.lastRunInfo = struct('isAdaptive', isAdaptive, 'duration', obj.duration, 'dt', obj.dt, ...
                'control_dt', obj.control_dt, 'adaptation_dt', obj.adaptation_dt, 'runName', obj.runName, ...
                'executionStartedAt', obj.executionStartedAt, 'executionFinishedAt', obj.executionFinishedAt, ...
                'executionElapsedSeconds', elapsedWallSeconds);
        end

        function persistCurrentRun(obj)
            %PERSISTCURRENTRUN Save finalized run data to sim_data.mat.
            fth.io.ResultsManager.persistRun(obj.resultsDir, ...
                obj.lastLogs, obj.lastMetrics, obj.lastEst, obj.lastRunInfo, obj.cfg);
        end

        function persistMetricsFile(obj)
            %PERSISTMETRICSFILE Save lightweight metrics.txt for every run.
            fth.io.ResultsManager.writeMetricsFile(obj.resultsDir, ...
                obj.lastMetrics, obj.lastRunInfo, obj.cfg);
        end

        function releaseRunMemory(obj)
            %RELEASERUNMEMORY Clear heavy in-memory run state after persistence.
            %   lastLogs is intentionally kept so getLogs() works after saveSimData=false.
            obj.lastMetrics = [];
            obj.lastEst = [];
            obj.massLog = [];
            obj.comLog = [];
            obj.inertiaLog = [];
            obj.estTimeLog = [];
            obj.log = [];
        end

        function plotCurrentRun(obj, plotType, displayPlots)
            %PLOTCURRENTRUN Plot finalized in-memory run data and save PNGs.
            fth.io.ResultsManager.plotRunData(obj.resultsDir, obj.lastLogs, obj.lastEst, ...
                obj.lastRunInfo, obj.cfg, plotType, displayPlots);
        end

        function plotSavedRun(~, resultsDir, plotType, displayPlots)
            %PLOTSAVEDRUN Generate plots for one saved run directory.
            fth.io.ResultsManager.plotSavedRun(resultsDir, plotType, displayPlots);
        end

        function [isAdaptive, payloadMass, payloadCoG, payloadDropTime, ...
                plotType, displayPlots, saveSimData] = parseRunInputs(obj, varargin)
            %PARSERUNINPUTS Parse run inputs plus post-run options.
            if isfield(obj.cfg.controller, 'adaptation')
                isAdaptive = ~strcmpi(obj.cfg.controller.adaptation, 'none');
            else
                isAdaptive = false;
            end
            payloadMass = obj.getPayloadField('mass', 0);
            payloadCoG = obj.getPayloadField('CoG', [0;0;0]);
            payloadDropTime = obj.getPayloadField('dropTime', inf);
            plotType = 'none';
            displayPlots = false;
            saveSimData = false;

            args = varargin;

            % Struct-based call: sim.run(opts) where opts has .plotMode, .displayPlots, .saveSimData.
            if ~isempty(args) && isstruct(args{1})
                opts = args{1};
                if isfield(opts, 'plotMode')    && ~isempty(opts.plotMode),    plotType     = opts.plotMode;     end
                if isfield(opts, 'displayPlots')&& ~isempty(opts.displayPlots),displayPlots = opts.displayPlots; end
                if isfield(opts, 'saveSimData') && ~isempty(opts.saveSimData), saveSimData  = opts.saveSimData;  end
                args = {};
            end

            if ~isempty(args) && ~obj.isPlotSpecifier(args{1})
                positionalCount = min(4, numel(args));
                if positionalCount >= 1 && ~isempty(args{1})
                    isAdaptive = args{1};
                end
                if positionalCount >= 2 && ~isempty(args{2})
                    payloadMass = args{2};
                end
                if positionalCount >= 3 && ~isempty(args{3})
                    payloadCoG = args{3};
                end
                if positionalCount >= 4 && ~isempty(args{4})
                    payloadDropTime = args{4};
                end
                args = args(positionalCount+1:end);
            end

            if ~isempty(args)
                plotType = args{1};
                args = args(2:end);
            end
            if ~isempty(args)
                displayPlots = args{1};
                args = args(2:end);
            end
            if ~isempty(args)
                saveSimData = args{1};
            end

            plotType = lower(string(plotType));
            if plotType ~= "none" && plotType ~= "summary" && plotType ~= "all"
                error("plotType must be 'none', 'summary', or 'all'.");
            end
            displayPlots = logical(displayPlots);
            saveSimData = logical(saveSimData);
            payloadCoG = payloadCoG(:);
        end

        function tf = isPlotSpecifier(~, value)
            %ISPLOTSPECIFIER Return true for a plotType string argument.
            tf = (ischar(value) || (isstring(value) && isscalar(value))) ...
                && any(strcmpi(string(value), ["none", "summary", "all"]));
        end

        function initCfg = getParamInit(obj)
            %GETPARAMINIT Read controller parameter init config with fallback.
            initCfg = struct('mode', 'vehicle', 'spec', []);
            if isprop(obj.cfg, 'controller') && isfield(obj.cfg.controller, 'paramInit')
                candidate = obj.cfg.controller.paramInit;
                if isstruct(candidate) && isfield(candidate, 'mode') && ~isempty(candidate.mode)
                    initCfg = candidate;
                end
            end
        end

        function est = getEstimationData(obj, logs)
            %GETESTIMATIONDATA Assemble estimation time series.
            %   Inputs: logs struct for time vector fallback.
            %   Output: est struct of estimated and actual parameters.
            est = struct();
            if ~isempty(obj.massLog)
                t = obj.estTimeLog(:);
                est.mass = obj.massLog(:);
                est.com = obj.comLog;
                est.inertia = obj.inertiaLog;
            else
                t = logs.t(:);
                est.mass = ones(numel(t), 1);
                est.com = zeros(numel(t), 3);
                est.inertia = zeros(numel(t), 6);
            end

            est.t = t;
            est.identifiability = obj.ctrl.getAdaptationDiagnostics();

            m_base   = obj.cfg.vehicle.m;
            I_base   = obj.cfg.vehicle.I_params;
            cog_base = obj.cfg.vehicle.CoG(:);
            m_base_scalar = m_base;
            I_base_row = I_base(:).';
            cog_base_col = cog_base(:);
            cog_base_row = cog_base_col.';

            m_with = m_base_scalar;
            I_with = I_base_row;
            cog_with = cog_base_row;
            if obj.payloadMass > 0
                [m_with, I_with, cog_with] = fth.utils.addPayload( ...
                    m_base_scalar, I_base_row, cog_base_col, obj.payloadMass, obj.payloadCoG);
                I_with = I_with(:).';
                cog_with = cog_with(:).';
            end

            if obj.payloadMass > 0 && isfinite(obj.payloadDropTime)
                est.dropTime = obj.payloadDropTime;
                est.massActual = m_with * ones(numel(t), 1);
                est.comActual = repmat(cog_with, numel(t), 1);
                est.inertiaActual = repmat(I_with, numel(t), 1);

                idx = t >= obj.payloadDropTime;
                est.massActual(idx) = m_base_scalar;
                est.comActual(idx,:) = repmat(cog_base_row, sum(idx), 1);
                est.inertiaActual(idx,:) = repmat(I_base_row, sum(idx), 1);
            else
                est.massActual = m_with * ones(numel(t), 1);
                est.comActual = repmat(cog_with, numel(t), 1);
                est.inertiaActual = repmat(I_with, numel(t), 1);
            end
        end
    end
end
