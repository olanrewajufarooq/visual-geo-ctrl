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
    %     cfg = fth.sim.Config()
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
            obj.initParamInit();
            obj.initVisualization();
            
            % Default trajectory and controller (can be overridden)
            obj.setTrajectory('hover');
            obj.setController('PD');
        end
        
        %% Setters (Fluent Interface)
        
        function obj = setTrajectory(obj, name, goToHoverBeforePathStarts)
            %SETTRAJECTORY Configure the reference trajectory type.
            %   name: 'circle','hover','infinity','lissajous3d','helix3d',
            %         'poly3d','takeoffland'
            %   goToHoverBeforePathStarts: logical flag, scalar or one-per-trajectory (optional)
            %
            %   Output:
            %     obj - Config instance (for chaining).

            obj.initTrajectory();
            trajNames = fth.sim.ConfigUtils.normalizeNames(name);
            if nargin > 2
                hoverInput = goToHoverBeforePathStarts;
                hasHover = true;
            else
                hoverInput = [];
                hasHover = false;
            end
            if hasHover
                trajHover = fth.sim.ConfigUtils.normalizeHover(hoverInput, numel(trajNames));
                obj.traj.batch = struct('names', {trajNames}, 'goToHoverBeforePathStarts', trajHover);
                obj.applyTrajectoryDefinition(trajNames{1}, trajHover(1));
            else
                obj.traj.batch = struct('names', {trajNames});
                obj.applyTrajectoryDefinition(trajNames{1});
            end
        end

        function obj = setController(obj, type, potential)
            %SETCONTROLLER Store controller type label and optional potential.
            %   type:      string label stored for logging/naming (not validated).
            %   potential: potType string (optional). Valid values:
            %              'log','inertia-gain','body-gain','ref-gain','sym-inv'.
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
                    potential = 'log';
                end
            end

            obj.controller.type = type;
            if ~isfield(obj.controller, 'adaptation') || isempty(obj.controller.adaptation)
                obj.controller.adaptation = 'none';
            end

            obj.controller.potential = potential;
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
                    obj.controller.Gamma = fth.sim.ConfigUtils.defaultGains(obj.controller.adaptation);
                end
            end
        end

        function obj = setAdaptation(obj, type)
            %SETADAPTATION Select parameter adaptation mode.
            %   type: 'none' | 'euclidean' | 'bregman'
            %
            %   Output:
            %     obj - Config instance (for chaining).
            if nargin < 2 || isempty(type)
                type = 'none';
            end
            validModes = {'none', 'euclidean', 'bregman'};
            type = lower(type);
            if ~ismember(type, validModes)
                error('fth:Config:InvalidAdaptationMode', ...
                    'Unknown adaptation mode ''%s''. Valid modes: %s', ...
                    type, strjoin(validModes, ', '));
            end
            obj.controller.adaptation = type;
            if ~strcmpi(type, 'none')
                if ~isfield(obj.controller, 'Gamma') || isempty(obj.controller.Gamma)
                    obj.controller.Gamma = fth.sim.ConfigUtils.defaultGains(type);
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
            %   Kp: 6-element vector (any orientation) for a single run, or
            %       N×6 / 6×N matrix for N batch runs.
            %
            %   Output:
            %     obj - Config instance (for chaining).
            if nargin < 2 || isempty(Kp)
                return;
            end
            [canonical, ~] = fth.sim.ConfigUtils.normalizeBatchField(Kp, 6);
            obj.controller.Kp = canonical;
        end

        function obj = setKdGains(obj, Kd)
            %SETKDGAINS Set derivative gains for one or more runs.
            %   Kd: 6-element vector (any orientation) for a single run, or
            %       N×6 / 6×N matrix for N batch runs.
            %
            %   Output:
            %     obj - Config instance (for chaining).
            if nargin < 2 || isempty(Kd)
                return;
            end
            [canonical, ~] = fth.sim.ConfigUtils.normalizeBatchField(Kd, 6);
            obj.controller.Kd = canonical;
        end

        function obj = setAdaptiveGains(obj, Gamma)
            %SETADAPTIVEGAINS Set adaptive gains for one or more runs.
            %   Bregman mode  : scalar or vector of scalars (one per batch run;
            %                   any orientation; 0 = disabled for that run).
            %   Euclidean mode: scalar, 10-element vector, N×10 or 10×N matrix.
            %
            %   Output:
            %     obj - Config instance (for chaining).
            if nargin < 2 || isempty(Gamma)
                return;
            end
            if isscalar(Gamma)
                obj.controller.Gamma = Gamma;
                return;
            end
            adaptMode = 'none';
            if isfield(obj.controller, 'adaptation'), adaptMode = obj.controller.adaptation; end
            singleWidth = 1;
            if ~strcmpi(adaptMode, 'bregman'), singleWidth = 10; end
            [canonical, ~] = fth.sim.ConfigUtils.normalizeBatchField(Gamma, singleWidth);
            obj.controller.Gamma = canonical;
        end

        function batchCount = getBatchCount(obj)
            %GETBATCHCOUNT Return the number of simulation runs requested.
            gainBatchCount = 1;
            kpVal = []; if isfield(obj.controller, 'Kp'), kpVal = obj.controller.Kp; end
            kdVal = []; if isfield(obj.controller, 'Kd'), kdVal = obj.controller.Kd; end
            gVal  = []; if isfield(obj.controller, 'Gamma'), gVal = obj.controller.Gamma; end
            adaptMode = 'none'; if isfield(obj.controller, 'adaptation'), adaptMode = obj.controller.adaptation; end
            counts = [ ...
                fth.sim.ConfigUtils.gainBatchCount(kpVal, 'Kp', 6), ...
                fth.sim.ConfigUtils.gainBatchCount(kdVal, 'Kd', 6), ...
                fth.sim.ConfigUtils.gammaBatchCount(gVal, adaptMode)];
            batched = counts(counts > 1);
            if ~isempty(batched)
                gainBatchCount = batched(1);
            end
            if any(batched ~= gainBatchCount)
                error('Config:InconsistentBatchCounts', ...
                    'Kp, Kd, and Gamma batch counts must match when more than one run is requested.');
            end
            M = obj.resolveM();
            batchCount = M * obj.getTrajectoryBatchCount();
        end

        function cfgs = expandBatchConfigs(obj, parentResultsDir)
            %EXPANDBATCHCONFIGS Expand a batched config into per-run configs.
            if nargin < 2
                parentResultsDir = '';
            end

            M = obj.resolveM();
            [trajNames, trajHover, hasHoverOverride] = obj.getTrajectoryBatchEntries();
            N = numel(trajNames);
            batchCount = N * M;

            % Build run name list (from sim.batchNames or fallback).
            batchNames = {};
            if isfield(obj.sim, 'batchNames'), batchNames = obj.sim.batchNames; end

            % Build coriolisFactorization list (always returned as a cell array).
            forms = obj.getCoriolisFormBatchEntries();

            cfgs = cell(batchCount, 1);
            cfgIndex = 1;

            for trajIdx = 1:N
                currentTrajName = trajNames(trajIdx);
                for simIdx = 1:M
                    cfgCopy = obj.copy();
                    cfgCopy.sim.batchNames = {};   % child is a single run; prevent re-expansion

                    % Apply trajectory.
                    if hasHoverOverride
                        cfgCopy.applyTrajectoryDefinition(currentTrajName{1}, trajHover(trajIdx));
                        cfgCopy.traj.batch = struct('names', {currentTrajName}, ...
                            'goToHoverBeforePathStarts', trajHover(trajIdx));
                    else
                        cfgCopy.applyTrajectoryDefinition(currentTrajName{1});
                        cfgCopy.traj.batch = struct('names', {currentTrajName});
                    end

                    % Select gains for this simIdx.
                    cfgCopy.controller.Kp = fth.sim.ConfigUtils.selectRow(obj.controller.Kp, simIdx);
                    cfgCopy.controller.Kd = fth.sim.ConfigUtils.selectRow(obj.controller.Kd, simIdx);
                    if isfield(obj.controller, 'Gamma') && ~isempty(obj.controller.Gamma)
                        cfgCopy.controller.Gamma = fth.sim.ConfigUtils.selectGammaRow( ...
                            obj.controller.adaptation, obj.controller.Gamma, simIdx);
                    end

                    % Select coriolisFactorization for this simIdx.
                    if numel(forms) > 1
                        cfgCopy.controller.coriolisFactorization = forms{simIdx};
                    else
                        cfgCopy.controller.coriolisFactorization = forms{1};
                    end

                    % Select payload fields for this simIdx.
                    if isfield(obj.payload, 'mass') && numel(obj.payload.mass) > 1
                        cfgCopy.payload.mass = obj.payload.mass(simIdx);
                    end
                    if isfield(obj.payload, 'CoG') && size(obj.payload.CoG,1) == 3 && size(obj.payload.CoG,2) == 1
                        % scalar 3×1 — broadcast as-is
                    elseif isfield(obj.payload, 'CoG') && size(obj.payload.CoG,2) == 3 && size(obj.payload.CoG,1) > 1
                        cfgCopy.payload.CoG = obj.payload.CoG(simIdx,:)';
                    end
                    if isfield(obj.payload, 'dropTime') && numel(obj.payload.dropTime) > 1
                        cfgCopy.payload.dropTime = obj.payload.dropTime(simIdx);
                    end

                    % Build run name: from batchNames or fallback.
                    if ~isempty(batchNames) && simIdx <= numel(batchNames)
                        runName = batchNames{simIdx};
                    else
                        runName = sprintf('run_%03d', simIdx);
                    end
                    cfgCopy.sim.runName = runName;
                    cfgCopy.sim.batchRunIndex = simIdx;
                    cfgCopy.sim.globalBatchIndex = cfgIndex;

                    % Build results folder.
                    if ~isempty(parentResultsDir)
                        leafFolder = runName;
                        if N > 1
                            trajFolder = fth.sim.ConfigUtils.trajectoryFolderName(currentTrajName{1}, trajIdx);
                            cfgCopy.sim.resultsDirOverride = fullfile(parentResultsDir, trajFolder, leafFolder);
                        else
                            cfgCopy.sim.resultsDirOverride = fullfile(parentResultsDir, leafFolder);
                        end
                    end

                    cfgCopy.sim.captureConsoleExternally = batchCount > 1;
                    cfgs{cfgIndex} = cfgCopy;
                    cfgIndex = cfgIndex + 1;
                end
            end
        end

        function cfgCopy = copy(obj)
            %COPY Create a detached copy of the configuration.
            cfgCopy = fth.sim.Config();
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
            kpVal = []; if isfield(obj.controller, 'Kp'), kpVal = obj.controller.Kp; end
            fth.sim.ConfigUtils.validateGainShape(kpVal, 'Kp', 6);
            kdVal = []; if isfield(obj.controller, 'Kd'), kdVal = obj.controller.Kd; end
            fth.sim.ConfigUtils.validateGainShape(kdVal, 'Kd', 6);
            gVal = []; if isfield(obj.controller, 'Gamma'), gVal = obj.controller.Gamma; end
            adaptMode = 'none'; if isfield(obj.controller, 'adaptation'), adaptMode = obj.controller.adaptation; end
            if ~isempty(gVal)
                fth.sim.ConfigUtils.validateGammaShape(gVal, adaptMode);
            end
            [trajNames, trajHover, ~] = obj.getTrajectoryBatchEntries();
            fth.sim.ConfigUtils.validateTrajectoryBatch(trajNames, trajHover);
            M = obj.resolveM();
            % Validate that all batched fields agree on M.
            % Gain rows must be 1 (broadcast) or exactly M.
            gainRows = fth.sim.ConfigUtils.gainBatchCount(kpVal, 'Kp', 6);
            gammaRows = fth.sim.ConfigUtils.gammaBatchCount(gVal, adaptMode);
            for rowCount = [gainRows, gammaRows]
                if rowCount > 1 && rowCount ~= M
                    error('Config:InconsistentBatchCounts', ...
                        'Gain row count (%d) must match batch count M=%d.', rowCount, M);
                end
            end
            if isfield(obj.controller, 'coriolisFactorization') && ...
                    iscell(obj.controller.coriolisFactorization) && ...
                    numel(obj.controller.coriolisFactorization) > 1
                if numel(obj.controller.coriolisFactorization) ~= M
                    error('Config:InconsistentBatchCounts', ...
                        'coriolisFactorization cell array length (%d) must match batch count M=%d.', ...
                        numel(obj.controller.coriolisFactorization), M);
                end
            end
            if isfield(obj.payload, 'mass') && numel(obj.payload.mass) > 1
                if numel(obj.payload.mass) ~= M
                    error('Config:InconsistentBatchCounts', ...
                        'payload.mass length (%d) must match batch count M=%d.', numel(obj.payload.mass), M);
                end
            end
            if isfield(obj.payload, 'CoG') && size(obj.payload.CoG,1) > 1 && size(obj.payload.CoG,2) == 3
                if size(obj.payload.CoG,1) ~= M
                    error('Config:InconsistentBatchCounts', ...
                        'payload.CoG row count (%d) must match batch count M=%d.', size(obj.payload.CoG,1), M);
                end
            end
            if isfield(obj.payload, 'dropTime') && numel(obj.payload.dropTime) > 1
                if numel(obj.payload.dropTime) ~= M
                    error('Config:InconsistentBatchCounts', ...
                        'payload.dropTime length (%d) must match batch count M=%d.', numel(obj.payload.dropTime), M);
                end
            end
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
                    obj.controller.Gamma = fth.sim.ConfigUtils.defaultGains(obj.controller.adaptation);
                end
            end
            if ~isfield(obj.controller, 'lambda') || isempty(obj.controller.lambda)
                obj.controller.lambda = zeros(6,1);
            end
            if ~isfield(obj.controller, 'coriolisFactorization') || isempty(obj.controller.coriolisFactorization)
                obj.controller.coriolisFactorization = 'basic';
            end
            if ~isfield(obj.controller, 'potential') || isempty(obj.controller.potential)
                obj.controller.potential = 'log';
            end
            obj.validateBatchGains();
        end

        function obj = setPayloadScenario(obj, mass, cog, dropTime)
            %SETPAYLOADSCENARIO Configure payload mass, CoG, and drop timing.
            %   mass    : scalar or M-element vector (any orientation).
            %   cog     : 3-element vector (any orientation) for a single run,
            %             or N×3 / 3×N matrix for N batch runs.
            %   dropTime: scalar or M-element vector (any orientation).
            %
            %   Output:
            %     obj - Config instance (for chaining).
            if nargin > 1
                [obj.payload.mass, ~] = fth.sim.ConfigUtils.normalizeBatchField(mass, 1);
            end
            if nargin > 2
                [obj.payload.CoG, ~] = fth.sim.ConfigUtils.normalizeBatchField(cog, 3);
            end
            if nargin > 3
                [obj.payload.dropTime, ~] = fth.sim.ConfigUtils.normalizeBatchField(dropTime, 1);
            end
        end

        function obj = setParamInit(obj, mode, spec)
            %SETPARAMINIT Configure controller parameter initialization.
            %   mode: 'vehicle', 'vehicle-plus-payload', 'mid-vehicle-payload',
            %         'vehicle-plus-payload-higher', 'vehicle-slight-dev', 'random',
            %         or a 10x1/1x10 custom theta vector.
            %   spec: optional mode-specific data (custom theta, random seed, etc.)
            %
            %   Output:
            %     obj - Config instance (for chaining).
            if nargin < 2 || isempty(mode)
                mode = 'vehicle';
            end
            if nargin < 3
                spec = [];
            end

            if isnumeric(mode)
                validateattributes(mode, {'numeric'}, {'vector', 'numel', 10}, '', 'mode');
                obj.controller.paramInit = struct('mode', 'custom', 'spec', mode(:));
                return;
            end

            mode = char(lower(string(mode)));
            validModes = {'vehicle', 'vehicle-plus-payload', 'mid-vehicle-payload', ...
                'vehicle-plus-payload-higher', 'vehicle-slight-dev', 'random', 'custom'};
            if ~ismember(mode, validModes)
                error('Config:InvalidParamInitMode', ...
                    'Parameter init mode must be one of: %s.', strjoin(validModes, ', '));
            end
            if ~isempty(spec) && ~strcmp(mode, 'random')
                validateattributes(spec, {'numeric'}, {'vector', 'numel', 10}, '', 'spec');
                spec = spec(:);
            end
            obj.controller.paramInit = struct('mode', mode, 'spec', spec);
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
        
        function obj = setPotentialType(obj, potential)
            %SETPOTENTIALTYPE Select the controller potential model.
            %   potential: 'log','inertia-gain','body-gain','ref-gain','sym-inv'.
            %
            %   Output:
            %     obj - Config instance (for chaining).
            obj.controller.potential = potential;
        end

        function obj = setLambda(obj, lambda)
            %SETLAMBDA Set the composite-variable coupling gain lambda.
            %   lambda: scalar or 6x1 vector (diagonal of the 6x6 Lambda matrix).
            %   Default is zeros(6,1) — Lambda = 0, so s = Ve.
            %
            %   Output:
            %     obj - Config instance (for chaining).
            if nargin < 2 || isempty(lambda)
                return;
            end
            if isscalar(lambda)
                obj.controller.lambda = lambda * ones(6,1);
            else
                obj.controller.lambda = lambda(:);
            end
        end

        function obj = setCoriolisFactorizationForm(obj, form)
            %SETCORIOLISFACTORIZATIONFORM Select the Coriolis factorization.
            %   form: 'basic' or 'consistent'. Pass a cell array for batch
            %   runs, e.g. {'basic','consistent'}.
            %
            %   Output:
            %     obj - Config instance (for chaining).
            if nargin < 2 || isempty(form)
                return;
            end
            if iscell(form)
                obj.controller.coriolisFactorization = cellfun(@lower, form, 'UniformOutput', false);
            else
                obj.controller.coriolisFactorization = lower(char(form));
            end
        end

        function obj = useTrajectoryOptions(obj, opts)
            %USETRAJECTORYOPTIONS Apply trajectory settings from a struct.
            %   Accepts the same fields as fth.plot.TrajPlotter.run(opts).
            %   Recognised fields:
            %     .name / .names              - trajectory name or cell array of names
            %     .goToHoverBeforePathStarts  - logical; override hover flag
            %     .scale                      - path scale [m]
            %     .altitude                   - hover altitude [m]
            %     .period                     - trajectory cycle duration [s]; if set,
            %                                   overrides duration-based period from done()
            %     .goToHoverDuration          - hover climb duration [s]
            %     .goToHoverPeriod            - alias for goToHoverDuration
            %
            %   Output:
            %     obj - Config instance (for chaining).

            % Resolve name(s).
            name = [];
            if isfield(opts, 'names') && ~isempty(opts.names)
                name = opts.names;
            elseif isfield(opts, 'name') && ~isempty(opts.name)
                name = opts.name;
            end

            % Forward to setTrajectory.
            if ~isempty(name)
                hasHover = isfield(opts, 'goToHoverBeforePathStarts') && ...
                               ~isempty(opts.goToHoverBeforePathStarts);
                if hasHover
                    obj.setTrajectory(name, opts.goToHoverBeforePathStarts);
                else
                    obj.setTrajectory(name);
                end
            elseif isfield(opts, 'goToHoverBeforePathStarts') && ~isempty(opts.goToHoverBeforePathStarts)
                obj.traj.goToHoverBeforePathStarts = logical(opts.goToHoverBeforePathStarts);
            end

            % Override individual trajectory parameters.
            if isfield(opts, 'scale')   && ~isempty(opts.scale),   obj.traj.scale   = opts.scale;   end
            if isfield(opts, 'altitude')&& ~isempty(opts.altitude), obj.traj.altitude= opts.altitude;end
            if isfield(opts, 'period')  && ~isempty(opts.period)
                obj.traj.period      = opts.period;
                obj.traj.useDuration = false;   % lock period; don't overwrite in done()
            end
            % goToHoverDuration / goToHoverPeriod (alias) — both map to cfg.traj.goToHoverDuration.
            if isfield(opts, 'goToHoverDuration') && ~isempty(opts.goToHoverDuration)
                obj.traj.goToHoverDuration = opts.goToHoverDuration;
            elseif isfield(opts, 'goToHoverPeriod') && ~isempty(opts.goToHoverPeriod)
                obj.traj.goToHoverDuration = opts.goToHoverPeriod;
            end
        end

        function obj = useControllerOptions(obj, opts)
            %USECONTROLLEROPTIONS Apply controller settings from a struct.
            %   Recognised fields:
            %     .type         - controller type label (string, stored for naming)
            %     .potential    - potType: 'log','inertia-gain','body-gain','ref-gain','sym-inv'
            %     .lambda       - scalar or 6x1 coupling gain
            %     .coriolisForm - Coriolis factorization: 'basic' or 'consistent';
            %                    cell array {'basic','consistent'} for batch runs
            %     .Kp           - 6x1 proportional gains
            %     .Kd           - 6x1 derivative gains
            %     .paramInit    - controller parameter init mode (works for nominal and adaptive);
            %                    'vehicle','vehicle-plus-payload','mid-vehicle-payload',
            %                    'vehicle-plus-payload-higher','vehicle-slight-dev','random',
            %                    or a 10x1 custom theta vector
            %
            %   Output:
            %     obj - Config instance (for chaining).
            if isfield(opts, 'type'),        obj.setController(opts.type);                        end
            if isfield(opts, 'potential'),   obj.setPotentialType(opts.potential);                end
            if isfield(opts, 'lambda'),      obj.setLambda(opts.lambda);                         end
            if isfield(opts, 'coriolisForm'),obj.setCoriolisFactorizationForm(opts.coriolisForm); end
            if isfield(opts, 'Kp'),          obj.setKpGains(opts.Kp);                            end
            if isfield(opts, 'Kd'),          obj.setKdGains(opts.Kd);                            end
            if isfield(opts, 'paramInit'),   obj.setParamInit(opts.paramInit);                   end
        end

        function obj = useAdaptationOptions(obj, opts)
            %USEADAPTATIONOPTIONS Apply adaptation settings from a struct.
            %   Recognised fields:
            %     .type   - adaptation mode: 'none','euclidean','bregman'
            %     .Gamma  - scalar or 10x1 adaptive gains
            %     .dt     - adaptation timestep [s]
            %
            %   Output:
            %     obj - Config instance (for chaining).
            if isfield(opts, 'type'),  obj.setAdaptation(opts.type);     end
            if isfield(opts, 'Gamma'), obj.setAdaptiveGains(opts.Gamma); end
            if isfield(opts, 'dt'),    obj.setAdaptationParams(opts.dt); end
        end

        function obj = usePayloadOptions(obj, opts)
            %USEPAYLOADOPTIONS Apply payload settings from a struct.
            %   Recognised fields:
            %     .mass     - payload mass [kg], scalar or M-element vector
            %     .CoG      - 3x1 payload CoG offset [m], or M×3 batch
            %     .dropTime - drop time [s], scalar or M-element vector
            %
            %   Output:
            %     obj - Config instance (for chaining).
            if isfield(opts, 'mass')
                [obj.payload.mass, ~] = fth.sim.ConfigUtils.normalizeBatchField(opts.mass, 1);
            end
            if isfield(opts, 'CoG')
                [obj.payload.CoG, ~] = fth.sim.ConfigUtils.normalizeBatchField(opts.CoG, 3);
            end
            if isfield(opts, 'dropTime')
                [obj.payload.dropTime, ~] = fth.sim.ConfigUtils.normalizeBatchField(opts.dropTime, 1);
            end
        end

        function obj = useSimOptions(obj, opts)
            %USESIMOPTIONS Apply simulation timing settings from a struct.
            %   Recognised fields:
            %     .dt           - simulation integration timestep [s]
            %     .duration     - total run time [s] (required together with .dt)
            %     .controlDt    - controller update period [s]
            %     .adaptationDt - adaptation update period [s]
            %     .names        - M-element cellstr of run names for batch labelling
            %
            %   Output:
            %     obj - Config instance (for chaining).
            if isfield(opts, 'dt') && isfield(opts, 'duration')
                obj.setSimParams(opts.dt, opts.duration);
            end
            if isfield(opts, 'controlDt'),    obj.setControlParams(opts.controlDt);       end
            if isfield(opts, 'adaptationDt'), obj.setAdaptationParams(opts.adaptationDt); end
            if isfield(opts, 'names') && ~isempty(opts.names)
                obj.sim.batchNames = fth.sim.ConfigUtils.normalizeNames(opts.names);
            end
        end

        function obj = useVizOptions(obj, opts)
            %USEVIZOPTIONSS Apply visualization settings from a struct.
            %   Recognised fields:
            %     .enable      - logical; toggle live view
            %     .liveSummary - logical; show live summary plots
            %     .updateRate  - update interval in control steps
            %     .embedUrdf   - logical; embed URDF in live view
            %     .plotLayout  - 'column-major' or 'row-major'
            %
            %   Output:
            %     obj - Config instance (for chaining).
            if isfield(opts, 'enable'),      obj.enableLiveView(opts.enable);           end
            if isfield(opts, 'liveSummary'), obj.setLiveSummary(opts.liveSummary);      end
            if isfield(opts, 'updateRate'),  obj.setLiveUpdateRate(opts.updateRate);    end
            if isfield(opts, 'embedUrdf'),   obj.setLiveUrdfEmbedding(opts.embedUrdf);  end
            if isfield(opts, 'plotLayout'),  obj.setPlotLayout(opts.plotLayout);        end
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

            obj.traj.period = duration;
        end

        function initVehicle(obj)
            %INITVEHICLE Initialize vehicle mass and inertia parameters.
            %   Populates vehicle mass, CoG, inertia parameters, and I6.
            obj.vehicle.g = 9.8;
            obj.vehicle.m = 3.646;
            obj.vehicle.CoG = [0; 0; -0.00229];
            obj.vehicle.I_params = [0.04092, 0.04017, 0.06921, 5.656e-5, 1.313e-5, -6.494e-5];
            % Compute I6 immediately using utility
            obj.vehicle.I6 = fth.se3.getGeneralizedInertia(obj.vehicle.m, obj.vehicle.I_params, obj.vehicle.CoG);
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

        function initParamInit(obj)
            %INITPARAMINIT Initialize controller parameter init to vehicle defaults.
            obj.controller.paramInit = struct('mode', 'vehicle', 'spec', []);
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
            if ~isfield(obj.traj, 'altitude') || isempty(obj.traj.altitude)
                obj.traj.altitude = 5;
            end
            if ~isfield(obj.traj, 'useDuration') || isempty(obj.traj.useDuration)
                obj.traj.useDuration = true;
            end
            if ~isfield(obj.traj, 'batch') || ~isstruct(obj.traj.batch) ...
                    || ~isfield(obj.traj.batch, 'names') || isempty(obj.traj.batch.names)
                trajName = 'hover';
                if isfield(obj.traj, 'name') && ~isempty(obj.traj.name)
                    trajName = char(string(obj.traj.name));
                end
                obj.traj.batch = struct('names', {{trajName}});
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

        function count = getSharedGainBatchCount(obj)
            %GETSHAREDGAINBATCHCOUNT Return the shared gain batch size.
            count = 1;
            kpVal = []; if isfield(obj.controller, 'Kp'), kpVal = obj.controller.Kp; end
            kdVal = []; if isfield(obj.controller, 'Kd'), kdVal = obj.controller.Kd; end
            gVal  = []; if isfield(obj.controller, 'Gamma'), gVal = obj.controller.Gamma; end
            adaptMode = 'none'; if isfield(obj.controller, 'adaptation'), adaptMode = obj.controller.adaptation; end
            counts = [ ...
                fth.sim.ConfigUtils.gainBatchCount(kpVal, 'Kp', 6), ...
                fth.sim.ConfigUtils.gainBatchCount(kdVal, 'Kd', 6), ...
                fth.sim.ConfigUtils.gammaBatchCount(gVal, adaptMode)];
            batched = counts(counts > 1);
            if ~isempty(batched), count = batched(1); end
        end

        function M = resolveM(obj)
            %RESOLVEM Return the per-trajectory sim count (M).
            batchNames = {};
            if isfield(obj.sim, 'batchNames'), batchNames = obj.sim.batchNames; end

            if ~isempty(batchNames)
                M = numel(batchNames);
                return;
            end

            kpVal = []; if isfield(obj.controller, 'Kp'), kpVal = obj.controller.Kp; end
            kdVal = []; if isfield(obj.controller, 'Kd'), kdVal = obj.controller.Kd; end
            gVal  = []; if isfield(obj.controller, 'Gamma'), gVal = obj.controller.Gamma; end
            adaptMode = 'none';
            if isfield(obj.controller, 'adaptation'), adaptMode = obj.controller.adaptation; end
            cfVal = []; if isfield(obj.controller, 'coriolisFactorization'), cfVal = obj.controller.coriolisFactorization; end
            massVal = []; if isfield(obj.payload, 'mass'), massVal = obj.payload.mass; end
            cogVal  = []; if isfield(obj.payload, 'CoG'), cogVal = obj.payload.CoG; end
            dtVal   = []; if isfield(obj.payload, 'dropTime'), dtVal = obj.payload.dropTime; end

            % Domain-aware counting for gain fields (a 6×1 Kp vector = one
            % 6-DOF config, not 6 batch rows).
            counts = [ ...
                fth.sim.ConfigUtils.gainBatchCount(kpVal, 'Kp', 6), ...
                fth.sim.ConfigUtils.gainBatchCount(kdVal, 'Kd', 6), ...
                fth.sim.ConfigUtils.gammaBatchCount(gVal, adaptMode)];

            % Generic counting for non-gain batched fields.
            if iscell(cfVal) && numel(cfVal) > 1
                counts(end+1) = numel(cfVal);
            end
            if ~isempty(massVal) && numel(massVal) > 1
                counts(end+1) = numel(massVal);
            end
            if ~isempty(cogVal) && size(cogVal,1) > 1 && size(cogVal,2) == 3
                counts(end+1) = size(cogVal,1);
            end
            if ~isempty(dtVal) && numel(dtVal) > 1
                counts(end+1) = numel(dtVal);
            end

            M = max(counts);
        end

        function count = getTrajectoryBatchCount(obj)
            %GETTRAJECTORYBATCHCOUNT Return number of configured trajectories.
            [trajNames, ~, ~] = obj.getTrajectoryBatchEntries();
            count = numel(trajNames);
        end

        function forms = getCoriolisFormBatchEntries(obj)
            %GETCORIOLISFORMBATCHENTRIES Return Coriolis forms as cell array of strings.
            if isfield(obj.controller, 'coriolisFactorization') && ...
                    iscell(obj.controller.coriolisFactorization)
                forms = obj.controller.coriolisFactorization;
            elseif isfield(obj.controller, 'coriolisFactorization') && ...
                    ~isempty(obj.controller.coriolisFactorization)
                forms = {char(obj.controller.coriolisFactorization)};
            else
                forms = {'basic'};
            end
        end

        function [trajNames, trajHover, hasHoverOverride] = getTrajectoryBatchEntries(obj)
            %GETTRAJECTORYBATCHENTRIES Return trajectory names and hover flags.
            obj.initTrajectory();
            if isfield(obj.traj, 'batch') && isstruct(obj.traj.batch) ...
                    && isfield(obj.traj.batch, 'names') && ~isempty(obj.traj.batch.names)
                trajNames = obj.traj.batch.names;
                if isfield(obj.traj.batch, 'goToHoverBeforePathStarts') && ~isempty(obj.traj.batch.goToHoverBeforePathStarts)
                    trajHover = logical(obj.traj.batch.goToHoverBeforePathStarts);
                    hasHoverOverride = true;
                else
                    trajHover = [];
                    hasHoverOverride = false;
                end
            else
                trajNames = {char(string(obj.traj.name))};
                trajHover = [];
                hasHoverOverride = false;
            end
        end

        function applyTrajectoryDefinition(obj, name, goToHoverBeforePathStarts)
            %APPLYTRAJECTORYDEFINITION Apply a single trajectory preset.
            obj.traj.name = name;
            if ~isfield(obj.traj, 'useDuration') || isempty(obj.traj.useDuration)
                obj.traj.useDuration = true;
            end
            hasHoverOverride = nargin > 2 && ~isempty(goToHoverBeforePathStarts);

            switch lower(name)
                case 'circle'
                    obj.traj.scale = 5;
                    obj.traj.goToHoverBeforePathStarts = true;

                case 'hover'
                    obj.traj.scale = 0;
                    obj.traj.goToHoverBeforePathStarts = true;
                    obj.traj.useDuration = false;

                case 'infinity'
                    obj.traj.scale = 5;
                    obj.traj.goToHoverBeforePathStarts = true;

                case 'lissajous3d'
                    obj.traj.scale = 5;
                    obj.traj.goToHoverBeforePathStarts = true;

                case 'helix3d'
                    obj.traj.scale = 5;
                    obj.traj.goToHoverBeforePathStarts = true;

                case 'poly3d'
                    obj.traj.scale = 5;
                    obj.traj.goToHoverBeforePathStarts = true;

                case 'takeoffland'
                    obj.traj.scale = 5;
                    obj.traj.goToHoverBeforePathStarts = false;

                otherwise
                    error('Unknown trajectory: %s. Valid names: circle, hover, infinity, lissajous3d, helix3d, poly3d, takeoffland.', name);
            end
            if hasHoverOverride
                obj.traj.goToHoverBeforePathStarts = logical(goToHoverBeforePathStarts);
            end
        end

    end
end
