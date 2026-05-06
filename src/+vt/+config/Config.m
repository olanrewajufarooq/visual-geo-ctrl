classdef Config < handle
    %CONFIG Centralized configuration for hexacopter simulations.
    %   Stores vehicle, simulation, trajectory, controller, visualization,
    %   and payload settings as nested structs.
    %
    %   Core fields:
    %     vehicle  - mass, CoG, inertia parameters, gravity.
    %     sim      - dt values, duration, and safety settings.
    %     traj     - path type, cycles, timing, and profile options.
    %     controller - gains, potential type, adaptation mode.
    %     viz      - live view and plot layout options.
    %
    %   Typical use:
    %     cfg = vt.config.Config()
    %         .setTrajectory('circle')
    %         .setController('PD')
    %         .setSimParams(0.005, 30);
    
    properties
        vehicle = struct()
        sim = struct()
        traj = struct()
        controller = struct()
        viz = struct()
        payload = struct()
        act = struct()
    end
    
    methods
        function obj = Config()
            %CONFIG Build a configuration with baseline defaults.
            %   Initializes vehicle, simulation, trajectory, payload, and
            %   visualization settings.
            %
            %   Output:
            %     obj - Config instance with defaults and a hover trajectory.
            obj.initVehicle();
            obj.initSimulation();
            obj.initTrajectory();
            obj.initPayload();
            obj.initEstimateInitialization();
            obj.initVisualization();
            
            % Default trajectory and controller (can be overridden)
            obj.setTrajectory('hover');
            obj.setController('PD');
        end
        
        %% Setters (Fluent Interface)
        
        function obj = setTrajectory(obj, name, cycles, startWithHover)
            %SETTRAJECTORY Configure the reference trajectory type and cycle count.
            %   name: 'circle','hover','infinity','infinity3d','infinity3dmod',
            %         'lissajous3d','helix3d','poly3d','takeoffland'
            %   cycles: number of cycles to run (default 1)
            %   startWithHover: logical flag, scalar or one-per-trajectory (optional)
            %
            %   Output:
            %     obj - Config instance (for chaining).

            obj.initTrajectory();
            trajNames = obj.normalizeTrajectoryNames(name);
            if nargin > 2
                cycleInput = cycles;
                hasCycles = true;
            else
                cycleInput = [];
                hasCycles = false;
            end
            if nargin > 3
                hoverInput = startWithHover;
                hasHover = true;
            else
                hoverInput = [];
                hasHover = false;
            end
            trajCycles = obj.normalizeTrajectoryCycles(hasCycles, cycleInput, numel(trajNames));
            if hasHover
                trajHover = obj.normalizeTrajectoryHover(hoverInput, numel(trajNames));
                obj.traj.batch = struct('names', {trajNames}, 'cycles', trajCycles, 'startWithHover', trajHover);
                obj.applyTrajectoryDefinition(trajNames{1}, trajCycles(1), trajHover(1));
            else
                obj.traj.batch = struct('names', {trajNames}, 'cycles', trajCycles);
                obj.applyTrajectoryDefinition(trajNames{1}, trajCycles(1));
            end
        end

        function obj = setTrajectoryMethod(obj, method, lambda)
            %SETTRAJECTORYMETHOD Choose trajectory generator and optional gains.
            %   method: 'precomputed' or 'modelreference'
            %   lambda: 6x1 filter gains for model-reference trajectories
            %
            %   Output:
            %     obj - Config instance (for chaining).
            if nargin > 1 && ~isempty(method)
                obj.traj.method = lower(method);
            end
            if nargin > 2 && ~isempty(lambda)
                obj.traj.lambda = lambda(:);
            end
        end
        
        function obj = setController(obj, type, potential)
            %SETCONTROLLER Configure controller type and potential function.
            %   type: 'PD', 'FeedLin', 'Feedforward'
            %   potential: 'liealgebra' or 'separate' (optional)
            %
            %   Output:
            %     obj - Config instance (for chaining).

            if nargin < 3 || isempty(potential)
                if isfield(obj.controller, 'potential')
                    potential = obj.controller.potential;
                else
                    potential = [];
                end
                if isempty(potential)
                    potential = 'liealgebra';
                end
            end

            obj.controller.type = type;
            if ~isfield(obj.controller, 'adaptation') || isempty(obj.controller.adaptation)
                obj.controller.adaptation = 'none';
            end
            
            obj.controller.potential = potential;
            
            switch lower(type)
                case 'pd'
                    % Uses default gains or those explicitly setup later.
                    
                case 'feedlin'
                    % Uses default gains or those explicitly setup later.

                case 'feedforward'
                    % Uses default gains or those explicitly setup later.
                    
                otherwise
                    error('Unknown controller type: %s', type);
            end
        end

        function ensureDefaultGains(obj)
            %ENSUREDEFAULTGAINS Set default gains if they haven't been configured.
            %   Called from done() method to ensure gains have sensible defaults.
            %   Should only be called after all configuration is complete.
            if ~isfield(obj.controller, 'Kp') || isempty(obj.controller.Kp)
                if strcmpi(obj.controller.adaptation, 'none')
                    obj.controller.Kp = [5.5, 5.5, 5.5, 5.5, 5.5, 5.5]';
                else
                    % For adaptive controllers, we might want slightly different gains
                    obj.controller.Kp = 1.01*[5.5, 5.5, 5.5, 5.5, 5.5, 5.5]';
                end
            end
            if ~isfield(obj.controller, 'Kd') || isempty(obj.controller.Kd)
                obj.controller.Kd = [2.05, 2.05, 2.05, 2.05, 2.05, 2.05]';
            end
            if ~isfield(obj.controller, 'Gamma') || isempty(obj.controller.Gamma)
                if ~strcmpi(obj.controller.adaptation, 'none')
                    obj.controller.Gamma = 4e-3 * [20;20;30;1;1;1;90;30;30;60];
                end
            end
        end

        function obj = setAdaptation(obj, type)
            %SETADAPTATION Select parameter adaptation mode.
            %   type: 'none','euclidean','geo-aware','geo-enforced','euclidean-boxed'
            %
            %   Output:
            %     obj - Config instance (for chaining).
            if nargin < 2 || isempty(type)
                type = 'none';
            end
            obj.controller.adaptation = lower(type);
            if ~strcmpi(type, 'none')
                if ~isfield(obj.controller, 'Gamma') || isempty(obj.controller.Gamma)
                    obj.controller.Gamma = 4e-3 * [20;20;30;1;1;1;90;30;30;60];
                end
            end
        end
        
        function obj = setControlParams(obj, control_dt)
            %SETCONTROLPARAMS Set the controller update period [s].
            %   control_dt: controller update timestep in seconds.
            %
            %   Output:
            %     obj - Config instance (for chaining).
            if nargin < 2 || isempty(control_dt)
                return;
            end
            obj.sim.control_dt = control_dt;
        end

        function obj = setAdaptationParams(obj, adaptation_dt)
            %SETADAPTATIONPARAMS Set the adaptation update period [s].
            %   adaptation_dt: adaptation update timestep in seconds.
            %
            %   Output:
            %     obj - Config instance (for chaining).
            if nargin < 2 || isempty(adaptation_dt)
                return;
            end
            obj.sim.adaptation_dt = adaptation_dt;
            obj.sim.adaptation_dt_auto = false;
        end

        function obj = setKpGains(obj, Kp)
            %SETKPGAINS Set proportional gains for one or more runs.
            %   Kp: 6x1, 1x6, or Nx6 proportional gains.
            %
            %   Output:
            %     obj - Config instance (for chaining).
            if nargin < 2 || isempty(Kp)
                return;
            end
            if isvector(Kp) && numel(Kp) == 6
                obj.controller.Kp = Kp(:);
            elseif ismatrix(Kp) && size(Kp,2) == 6
                obj.controller.Kp = Kp;
            else
                warning('setKpGains: Invalid input, Kp must be a 6x1 vector, 1x6 vector, or Nx6 matrix.');
            end
        end

        function obj = setKdGains(obj, Kd)
            %SETKDGAINS Set derivative gains for one or more runs.
            %   Kd: 6x1, 1x6, or Nx6 derivative gains.
            %
            %   Output:
            %     obj - Config instance (for chaining).
            if nargin < 2 || isempty(Kd)
                return;
            end
            if isvector(Kd) && numel(Kd) == 6
                obj.controller.Kd = Kd(:);
            elseif ismatrix(Kd) && size(Kd,2) == 6
                obj.controller.Kd = Kd;
            else
                warning('setKdGains: Invalid input, Kd must be a 6x1 vector, 1x6 vector, or Nx6 matrix.');
            end
        end

        function obj = setAdaptiveGains(obj, Gamma)
            %SETADAPTIVEGAINS Set adaptive gains for one or more runs.
            %   Gamma: 10x1, 1x10, or Nx10 adaptive gains.
            %
            %   Output:
            %     obj - Config instance (for chaining).
            if nargin < 2 || isempty(Gamma)
                return;
            end
            if isvector(Gamma) && numel(Gamma) == 10
                obj.controller.Gamma = Gamma(:);
            elseif ismatrix(Gamma) && size(Gamma,2) == 10
                obj.controller.Gamma = Gamma;
            else
                warning('setAdaptiveGains: Invalid input, Gamma must be a 10x1 vector, 1x10 vector, or Nx10 matrix.');
            end
        end

        function batchCount = getBatchCount(obj)
            %GETBATCHCOUNT Return the number of simulation runs requested.
            gainBatchCount = 1;
            counts = [ ...
                obj.getGainBatchCount('Kp', 6), ...
                obj.getGainBatchCount('Kd', 6), ...
                obj.getGainBatchCount('Gamma', 10)];
            batched = counts(counts > 1);
            if ~isempty(batched)
                gainBatchCount = batched(1);
            end
            if any(batched ~= gainBatchCount)
                error('Config:InconsistentBatchCounts', ...
                    'Kp, Kd, and Gamma batch counts must match when more than one run is requested.');
            end
            batchCount = gainBatchCount * obj.getTrajectoryBatchCount();
        end

        function cfgs = expandBatchConfigs(obj, parentResultsDir)
            %EXPANDBATCHCONFIGS Expand a batched config into per-run configs.
            if nargin < 2
                parentResultsDir = '';
            end
            gainBatchCount = obj.getSharedGainBatchCount();
            [trajNames, trajCycles, trajHover, hasHoverOverride] = obj.getTrajectoryBatchEntries();
            batchCount = gainBatchCount * numel(trajNames);
            cfgs = cell(batchCount, 1);
            cfgIndex = 1;
            for trajIdx = 1:numel(trajNames)
                currentTrajName = trajNames(trajIdx);
                currentTrajCycles = trajCycles(trajIdx);
                for gainIdx = 1:gainBatchCount
                    cfgCopy = obj.copy();
                    if hasHoverOverride
                        cfgCopy.applyTrajectoryDefinition(currentTrajName{1}, currentTrajCycles, trajHover(trajIdx));
                        cfgCopy.traj.batch = struct( ...
                            'names', {currentTrajName}, ...
                            'cycles', currentTrajCycles, ...
                            'startWithHover', trajHover(trajIdx));
                    else
                        cfgCopy.applyTrajectoryDefinition(currentTrajName{1}, currentTrajCycles);
                        cfgCopy.traj.batch = struct( ...
                            'names', {currentTrajName}, ...
                            'cycles', currentTrajCycles);
                    end
                    cfgCopy.controller.Kp = obj.selectGainRow(obj.controller.Kp, gainIdx);
                    cfgCopy.controller.Kd = obj.selectGainRow(obj.controller.Kd, gainIdx);
                    if isfield(obj.controller, 'Gamma') && ~isempty(obj.controller.Gamma)
                        cfgCopy.controller.Gamma = obj.selectGainRow(obj.controller.Gamma, gainIdx);
                    end
                    if ~isempty(parentResultsDir)
                        runFolder = sprintf('run_%03d', gainIdx);
                        if numel(trajNames) > 1
                        trajFolder = obj.getTrajectoryFolderName(currentTrajName{1}, trajIdx);
                            cfgCopy.sim.resultsDirOverride = fullfile(parentResultsDir, trajFolder, runFolder);
                        else
                            cfgCopy.sim.resultsDirOverride = fullfile(parentResultsDir, runFolder);
                        end
                    end
                    cfgCopy.sim.captureConsoleExternally = batchCount > 1;
                    cfgCopy.sim.batchRunIndex = gainIdx;
                    cfgCopy.sim.globalBatchIndex = cfgIndex;
                    cfgs{cfgIndex} = cfgCopy;
                    cfgIndex = cfgIndex + 1;
                end
            end
        end

        function cfgCopy = copy(obj)
            %COPY Create a detached copy of the configuration.
            cfgCopy = vt.config.Config();
            cfgCopy.vehicle = obj.vehicle;
            cfgCopy.sim = obj.sim;
            cfgCopy.traj = obj.traj;
            cfgCopy.controller = obj.controller;
            cfgCopy.viz = obj.viz;
            cfgCopy.payload = obj.payload;
            cfgCopy.act = obj.act;
        end

        function obj = validateBatchGains(obj)
            %VALIDATEBATCHGAINS Validate configured gain shapes and counts.
            obj.validateGainShape('Kp', 6);
            obj.validateGainShape('Kd', 6);
            if isfield(obj.controller, 'Gamma') && ~isempty(obj.controller.Gamma)
                obj.validateGainShape('Gamma', 10);
            end
            obj.validateTrajectoryBatch();
            obj.getBatchCount();
        end

        function obj = setSimParams(obj, sim_dt, duration)
            %SETSIMPARAMS Set simulation integration step and duration.
            %   sim_dt: integration timestep in seconds.
            %   duration: total run time in seconds.
            %
            %   Output:
            %     obj - Config instance (for chaining).
            obj.initTrajectory();
            obj.sim.dt = sim_dt;
            obj.sim.sim_dt_auto = false;
            obj.sim.duration = duration;
        end

        function obj = done(obj)
            %DONE Normalize time steps and sync trajectory period.
            %   Call at the end of config setup to resolve dt values.
            %
            %   Output:
            %     obj - Config instance (for chaining).
            obj.normalizeTimeSteps(true);
            obj.syncTrajectoryPeriod();
            
            % Ensure gains have sensible defaults
            if ~isfield(obj.controller, 'Kp') || isempty(obj.controller.Kp)
                if strcmpi(obj.controller.adaptation, 'none')
                    obj.controller.Kp = [5.5, 5.5, 5.5, 5.5, 5.5, 5.5]';
                else
                    % For adaptive controllers, we might want slightly different gains
                    obj.controller.Kp = 1.01*[5.5, 5.5, 5.5, 5.5, 5.5, 5.5]';
                end
            end
            if ~isfield(obj.controller, 'Kd') || isempty(obj.controller.Kd)
                obj.controller.Kd = [2.05, 2.05, 2.05, 2.05, 2.05, 2.05]';
            end
            if ~isfield(obj.controller, 'Gamma') || isempty(obj.controller.Gamma)
                if ~strcmpi(obj.controller.adaptation, 'none')
                    obj.controller.Gamma = 4e-3 * [20;20;30;1;1;1;90;30;30;60];
                end
            end
            obj.validateBatchGains();
        end

        function obj = setPayloadScenario(obj, mass, cog, dropTime)
            %SETPAYLOADSCENARIO Configure payload mass, CoG, and drop timing.
            %   mass: payload mass in kg.
            %   cog: 3x1 payload center-of-gravity offset in meters.
            %   dropTime: seconds into the run to drop payload.
            %
            %   Output:
            %     obj - Config instance (for chaining).
            if nargin > 1
                obj.payload.mass = mass;
            end
            if nargin > 2
                obj.payload.CoG = cog(:);
            end
            if nargin > 3
                obj.payload.dropTime = dropTime;
            end
        end

        function obj = setPayload(obj, mass, cog, dropTime, varargin)
            %SETPAYLOAD Deprecated alias for setPayloadScenario.
            %   Use setPayloadScenario(...) and setEstimateInitialization(...)
            %   instead of overloading payload configuration with estimate
            %   initialization behavior.
            if nargin > 4 && ~isempty(varargin)
                error('Config:DeprecatedSetPayload', ...
                    ['setPayload no longer accepts an estimate-initialization flag. ' ...
                     'Use setPayloadScenario(...) and setEstimateInitialization(...).']);
            end
            obj.setPayloadScenario(mass, cog, dropTime);
        end

        function obj = setEstimateInitialization(obj, mode, spec)
            %SETESTIMATEINITIALIZATION Configure adaptive estimate startup.
            %   mode: 'nominal', 'true', 'fixed', 'fixed-higher', or
            %         'random', or a 10x1/1x10 custom theta vector for
            %         fixed initialization
            %   spec: optional mode-specific data
            %
            %   Output:
            %     obj - Config instance (for chaining).
            if nargin < 2 || isempty(mode)
                mode = 'nominal';
            end
            if nargin < 3
                spec = [];
            end

            if isnumeric(mode)
                validateattributes(mode, {'numeric'}, {'vector', 'numel', 10}, '', 'mode');
                obj.controller.estimateInitialization = struct('mode', 'fixed', 'spec', mode(:));
                return;
            end

            mode = char(lower(string(mode)));
            validModes = {'nominal', 'true', 'fixed', 'fixed-higher', 'random'};
            if ~ismember(mode, validModes)
                error('Config:InvalidEstimateInitializationMode', ...
                    'Estimate initialization mode must be one of: nominal, true, fixed, fixed-higher, random.');
            end
            if strcmp(mode, 'fixed') && ~isempty(spec)
                validateattributes(spec, {'numeric'}, {'vector', 'numel', 10}, '', 'spec');
                spec = spec(:);
            end
            obj.controller.estimateInitialization = struct('mode', mode, 'spec', spec);
        end
        
        function obj = setVisualization(obj, enable, dynamicAxis, padding, initialAxis)
            %SETVISUALIZATION Configure visualization toggles and axis behavior.
            %   enable: toggle live visualization.
            %   dynamicAxis: auto-resize axes when enabled.
            %   padding: extra padding for dynamic axis limits.
            %   initialAxis: initial axis limits or 'auto'.
            %
            %   Output:
            %     obj - Config instance (for chaining).
            if nargin > 2
                obj.viz.enable = enable;
            end
            if nargin > 3
                obj.viz.dynamicAxis = dynamicAxis;
            end
            if nargin > 4
                obj.viz.axisPadding = padding;
            end
            if nargin > 5
                obj.viz.initialAxis = initialAxis;
            end
        end
        
        function obj = setActuationMethod(obj, method)
            %SETACTUATIONMETHOD Set the actuation mapping strategy.
            %   method: string identifier for allocation/mapping.
            %
            %   Output:
            %     obj - Config instance (for chaining).
            obj.act.method = method;
        end
        
        function obj = setPotentialType(obj, potential)
            %SETPOTENTIALTYPE Select the controller potential model.
            %   potential: 'liealgebra' or 'separate'.
            %
            %   Output:
            %     obj - Config instance (for chaining).
            obj.controller.potential = potential;
        end

        function obj = enableLiveView(obj, enable)
            %ENABLELIVEVIEW Toggle live visualization.
            %   enable: true/false.
            %
            %   Output:
            %     obj - Config instance (for chaining).
            if nargin > 1
                obj.viz.enable = enable;
            end
        end

        function obj = setLiveSummary(obj, liveSummary)
            %SETLIVESUMMARY Toggle live summary plots.
            %   liveSummary: true/false.
            %
            %   Output:
            %     obj - Config instance (for chaining).
            if nargin > 1
                obj.viz.liveSummary = liveSummary;
            end
        end

        function obj = setLiveUpdateRate(obj, updateEvery)
            %SETLIVEUPDATERATE Set live plot update cadence (in control steps).
            %   updateEvery: update interval in control steps.
            %
            %   Output:
            %     obj - Config instance (for chaining).
            if nargin > 1
                obj.viz.updateEvery = updateEvery;
            end
        end

        function obj = setLiveUrdfEmbedding(obj, embedUrdf)
            %SETLIVEURDFEMBEDDING Control URDF embedding for the live view.
            %   embedUrdf: true to embed URDF, false to load from file.
            %
            %   Output:
            %     obj - Config instance (for chaining).
            if nargin > 1
                obj.viz.embedUrdf = embedUrdf;
            end
        end

        function obj = setPlotLayout(obj, layoutType)
            %SETPLOTLAYOUT Set plot grid layout: 'row-major' or 'column-major'.
            %   layoutType: layout string.
            %
            %   Output:
            %     obj - Config instance (for chaining).
            if nargin < 2 || isempty(layoutType)
                return;
            end
            layoutType = lower(string(layoutType));
            if layoutType ~= "row-major" && layoutType ~= "column-major"
                error("plotLayout must be 'row-major' or 'column-major'.");
            end
            obj.viz.plotLayout = char(layoutType);
        end
        
    end
    
    methods (Access = private)
        function syncTrajectoryPeriod(obj)
            %SYNCTRAJECTORYPERIOD Match trajectory period to sim duration.
            %   Uses sim.duration and traj.cycles to set traj.period.
            if ~isfield(obj.traj, 'useDuration') || ~obj.traj.useDuration
                return;
            end
            if ~isfield(obj.sim, 'duration') || isempty(obj.sim.duration)
                return;
            end

            duration = obj.sim.duration;
            if ~isfinite(duration) || duration <= 0
                return;
            end

            cycles = 1;
            if isfield(obj.traj, 'cycles') && ~isempty(obj.traj.cycles)
                cycles = max(obj.traj.cycles, 1);
            end

            obj.traj.cycles = cycles;
            obj.traj.period = duration / cycles;
        end

        function initVehicle(obj)
            %INITVEHICLE Initialize vehicle mass and inertia parameters.
            %   Populates vehicle mass, CoG, inertia parameters, and I6.
            obj.vehicle.g = 9.8;
            obj.vehicle.m = 3.646;
            obj.vehicle.CoG = [0; 0; -0.00229];
            obj.vehicle.I_params = [0.04092, 0.04017, 0.06921, 5.656e-5, -6.494e-5, 1.313e-5];
            % Compute I6 immediately using utility
            obj.vehicle.I6 = vt.utils.getGeneralizedInertia(obj.vehicle.m, obj.vehicle.I_params, obj.vehicle.CoG);
        end
        
        function initSimulation(obj)
            %INITSIMULATION Initialize simulation timing and safety defaults.
            %   Sets dt values, duration, and ground interaction parameters.
            obj.sim.control_dt = 0.005;
            obj.sim.adaptation_dt = obj.sim.control_dt / 5;
            obj.sim.dt = obj.sim.adaptation_dt;
            obj.sim.duration = 30;
            obj.sim.adaptation_dt_auto = true;
            obj.sim.sim_dt_auto = true;
            obj.sim.enableSafety = true;
            obj.sim.groundEnable = true;
            obj.sim.groundHeight = 0;
            obj.sim.groundStiffness = 5000;
            obj.sim.groundDamping = 200;
            obj.sim.groundFriction = 0.3;
            obj.sim.minZ = obj.sim.groundHeight - 0.2;
        end

        function initPayload(obj)
            %INITPAYLOAD Initialize payload parameters.
            %   Defaults to no payload and no drop event.
            obj.payload.mass = 0;
            obj.payload.CoG = [0; 0; 0];
            obj.payload.dropTime = inf;
        end

        function initEstimateInitialization(obj)
            %INITESTIMATEINITIALIZATION Initialize adaptive estimate startup mode.
            obj.controller.estimateInitialization = struct('mode', 'nominal', 'spec', []);
        end
        
        function initVisualization(obj)
            %INITVISUALIZATION Initialize visualization defaults.
            %   Enables live view and sets layout/padding defaults.
            obj.viz.enable = false;
            obj.viz.dynamicAxis = true;
            obj.viz.axisPadding = 2.0;
            obj.viz.initialAxis = 'auto';
            obj.viz.liveSummary = false;
            obj.viz.updateEvery = 10;
            obj.viz.embedUrdf = false;
            obj.viz.plotLayout = 'column-major';
        end

        function initTrajectory(obj)
            %INITTRAJECTORY Initialize trajectory defaults.
            %   Seeds trajectory method, altitude, and cycles.
            if ~isstruct(obj.traj)
                obj.traj = struct();
            end
            if ~isfield(obj.traj, 'method') || isempty(obj.traj.method)
                obj.traj.method = 'precomputed';
            end
            if ~isfield(obj.traj, 'altitude') || isempty(obj.traj.altitude)
                obj.traj.altitude = 5;
            end
            if ~isfield(obj.traj, 'hoverFrac') || isempty(obj.traj.hoverFrac)
                obj.traj.hoverFrac = 0.1;
            end
            if ~isfield(obj.traj, 'lambda') || isempty(obj.traj.lambda)
                obj.traj.lambda = ones(6,1);
            end
            if ~isfield(obj.traj, 'useDuration') || isempty(obj.traj.useDuration)
                obj.traj.useDuration = true;
            end
            if ~isfield(obj.traj, 'cycles') || isempty(obj.traj.cycles)
                obj.traj.cycles = 1;
            end
            if ~isfield(obj.traj, 'batch') || ~isstruct(obj.traj.batch) ...
                    || ~isfield(obj.traj.batch, 'names') || isempty(obj.traj.batch.names)
                trajName = 'hover';
                if isfield(obj.traj, 'name') && ~isempty(obj.traj.name)
                    trajName = char(string(obj.traj.name));
                end
                obj.traj.batch = struct('names', {{trajName}}, 'cycles', obj.traj.cycles);
            end
        end

        function normalizeTimeSteps(obj, strict)
            %NORMALIZETIMESTEPS Resolve dt values for control/adaptation/sim.
            %   strict: enforce dt ordering constraints if true.
            if nargin < 2
                strict = false;
            end
            if ~isfield(obj.sim, 'control_dt') || isempty(obj.sim.control_dt)
                obj.sim.control_dt = obj.sim.dt;
            end

            adaptationEnabled = isfield(obj.controller, 'adaptation') && ~strcmpi(obj.controller.adaptation, 'none');
            if ~isfield(obj.sim, 'adaptation_dt_auto')
                obj.sim.adaptation_dt_auto = true;
            end
            if ~isfield(obj.sim, 'sim_dt_auto')
                obj.sim.sim_dt_auto = true;
            end

            if adaptationEnabled
                if obj.sim.adaptation_dt_auto
                    obj.sim.adaptation_dt = obj.sim.control_dt / 5;
                end
            else
                if obj.sim.adaptation_dt_auto || strict
                    obj.sim.adaptation_dt = obj.sim.control_dt;
                end
            end

            if obj.sim.sim_dt_auto
                obj.sim.dt = obj.sim.adaptation_dt;
            end

            if strict
                if adaptationEnabled && obj.sim.adaptation_dt >= obj.sim.control_dt
                    error('adaptation_dt must be smaller than control_dt when adaptation is enabled.');
                end

                if obj.sim.dt > obj.sim.adaptation_dt
                    error('sim_dt must be equal to or smaller than adaptation_dt.');
                end
            end
        end

        function count = getGainBatchCount(obj, fieldName, expectedRows)
            %GETGAINBATCHCOUNT Return number of gain columns for a field.
            count = 1;
            if ~isfield(obj.controller, fieldName) || isempty(obj.controller.(fieldName))
                return;
            end
            value = obj.controller.(fieldName);
            obj.validateGainShape(fieldName, expectedRows);
            if ismatrix(value) && size(value,2) == expectedRows && size(value,1) > 1
                count = size(value,1);
            end
        end

        function validateGainShape(obj, fieldName, expectedRows)
            %VALIDATEGAINSHAPE Validate the row count for a configured gain field.
            if ~isfield(obj.controller, fieldName) || isempty(obj.controller.(fieldName))
                return;
            end
            value = obj.controller.(fieldName);
            isValidVector = isvector(value) && numel(value) == expectedRows;
            isValidMatrix = ismatrix(value) && size(value,2) == expectedRows;
            if ~(isValidVector || isValidMatrix)
                error('Config:InvalidGainShape', ...
                    '%s must be a %dx1 vector, 1x%d vector, or Nx%d matrix.', ...
                    fieldName, expectedRows, expectedRows, expectedRows);
            end
        end

        function value = selectGainRow(~, gainValue, index)
            %SELECTGAINROW Select or broadcast a gain row for a run index.
            if isvector(gainValue)
                value = gainValue(:);
            elseif size(gainValue,1) == 1
                value = gainValue(:);
            else
                value = gainValue(index,:).';
            end
        end

        function count = getSharedGainBatchCount(obj)
            %GETSHAREDGAINBATCHCOUNT Return the shared gain batch size.
            count = 1;
            counts = [ ...
                obj.getGainBatchCount('Kp', 6), ...
                obj.getGainBatchCount('Kd', 6), ...
                obj.getGainBatchCount('Gamma', 10)];
            batched = counts(counts > 1);
            if ~isempty(batched)
                count = batched(1);
            end
        end

        function count = getTrajectoryBatchCount(obj)
            %GETTRAJECTORYBATCHCOUNT Return number of configured trajectories.
            [trajNames, ~] = obj.getTrajectoryBatchEntries();
            count = numel(trajNames);
        end

        function [trajNames, trajCycles, trajHover, hasHoverOverride] = getTrajectoryBatchEntries(obj)
            %GETTRAJECTORYBATCHENTRIES Return trajectory names, cycles, and hover flags.
            obj.initTrajectory();
            if isfield(obj.traj, 'batch') && isstruct(obj.traj.batch) ...
                    && isfield(obj.traj.batch, 'names') && ~isempty(obj.traj.batch.names)
                trajNames = obj.traj.batch.names;
                trajCycles = obj.traj.batch.cycles;
                if isfield(obj.traj.batch, 'startWithHover') && ~isempty(obj.traj.batch.startWithHover)
                    trajHover = logical(obj.traj.batch.startWithHover);
                    hasHoverOverride = true;
                else
                    trajHover = [];
                    hasHoverOverride = false;
                end
            else
                trajNames = {char(string(obj.traj.name))};
                trajCycles = obj.traj.cycles;
                trajHover = [];
                hasHoverOverride = false;
            end
        end

        function validateTrajectoryBatch(obj)
            %VALIDATETRAJECTORYBATCH Validate trajectory batch configuration.
            [trajNames, trajCycles, trajHover, hasHoverOverride] = obj.getTrajectoryBatchEntries();
            if isempty(trajNames)
                error('Config:InvalidTrajectoryBatch', 'At least one trajectory must be configured.');
            end
            if numel(trajCycles) ~= numel(trajNames)
                error('Config:InvalidTrajectoryCycles', ...
                    'Trajectory cycles must be a scalar or match the number of trajectories.');
            end
            if hasHoverOverride && numel(trajHover) ~= numel(trajNames)
                error('Config:InvalidTrajectoryHover', ...
                    'Trajectory startWithHover must be a scalar or match the number of trajectories.');
            end
        end

        function applyTrajectoryDefinition(obj, name, cycles, startWithHover)
            %APPLYTRAJECTORYDEFINITION Apply a single trajectory preset.
            obj.traj.name = name;
            obj.traj.cycles = cycles;
            obj.traj.useDuration = true;
            hasHoverOverride = nargin > 3 && ~isempty(startWithHover);

            switch lower(name)
                case 'circle'
                    obj.traj.scale = 5;
                    obj.traj.startWithHover = true;

                case 'hover'
                    obj.traj.scale = 0;
                    obj.traj.startWithHover = true;
                    obj.traj.useDuration = false;

                case 'infinity'
                    obj.traj.scale = 5;
                    obj.traj.startWithHover = true;

                case 'infinity3d'
                    obj.traj.scale = 5;
                    obj.traj.startWithHover = true;

                case 'infinity3dmod'
                    obj.traj.scale = 5;
                    obj.traj.startWithHover = true;

                case 'lissajous3d'
                    obj.traj.scale = 5;
                    obj.traj.startWithHover = true;

                case 'helix3d'
                    obj.traj.scale = 5;
                    obj.traj.startWithHover = true;

                case 'poly3d'
                    obj.traj.scale = 5;
                    obj.traj.startWithHover = true;

                case 'takeoffland'
                    obj.traj.scale = 5;
                    obj.traj.startWithHover = false;

                otherwise
                    error('Unknown trajectory: %s', name);
            end
            if hasHoverOverride
                obj.traj.startWithHover = logical(startWithHover);
            end

            obj.syncTrajectoryPeriod();
        end

        function trajNames = normalizeTrajectoryNames(~, name)
            %NORMALIZETRAJECTORYNAMES Normalize trajectory input into a cellstr.
            if ischar(name) || (isstring(name) && isscalar(name))
                trajNames = {char(string(name))};
            elseif isstring(name)
                trajNames = cellstr(name(:));
            elseif iscell(name)
                trajNames = cellfun(@char, cellfun(@string, name(:), 'UniformOutput', false), 'UniformOutput', false);
            else
                error('Config:InvalidTrajectoryInput', ...
                    'Trajectory must be a char, string scalar, string array, or cell array.');
            end
            trajNames = cellfun(@strtrim, trajNames, 'UniformOutput', false);
            if any(cellfun(@isempty, trajNames))
                error('Config:InvalidTrajectoryInput', 'Trajectory names cannot be empty.');
            end
        end

        function trajCycles = normalizeTrajectoryCycles(~, hasCycles, cycles, count)
            %NORMALIZETRAJECTORYCYCLES Normalize trajectory cycle input.
            if ~hasCycles || isempty(cycles)
                trajCycles = ones(1, count);
                return;
            end
            if isscalar(cycles)
                trajCycles = repmat(double(cycles), 1, count);
                return;
            end
            trajCycles = double(cycles(:)).';
            if numel(trajCycles) ~= count
                error('Config:InvalidTrajectoryCycles', ...
                    'Trajectory cycles must be a scalar or match the number of trajectories.');
            end
        end

        function trajHover = normalizeTrajectoryHover(~, startWithHover, count)
            %NORMALIZETRAJECTORYHOVER Normalize trajectory hover input.
            if isscalar(startWithHover)
                trajHover = repmat(logical(startWithHover), 1, count);
                return;
            end
            trajHover = logical(startWithHover(:)).';
            if numel(trajHover) ~= count
                error('Config:InvalidTrajectoryHover', ...
                    'Trajectory startWithHover must be a scalar or match the number of trajectories.');
            end
        end

        function folderName = getTrajectoryFolderName(~, name, index)
            %GETTRAJECTORYFOLDERNAME Build a compact trajectory folder name.
            shortName = vt.sim.NamingUtils.trajectoryLabel(name);
            folderName = sprintf('t%02d_%s', index, shortName);
        end
    end
end
