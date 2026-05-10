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
            trajNames = obj.normalizeTrajectoryNames(name);
            if nargin > 2
                hoverInput = goToHoverBeforePathStarts;
                hasHover = true;
            else
                hoverInput = [];
                hasHover = false;
            end
            if hasHover
                trajHover = obj.normalizeTrajectoryHover(hoverInput, numel(trajNames));
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
                    obj.controller.Gamma = 4e-3 * [20;20;30;1;1;1;90;30;30;60];
                end
            end
        end

        function obj = setAdaptation(obj, type)
            %SETADAPTATION Select parameter adaptation mode.
            %   type: 'none' | 'euclidean' | 'geo-aware' (not yet implemented)
            %
            %   Output:
            %     obj - Config instance (for chaining).
            if nargin < 2 || isempty(type)
                type = 'none';
            end
            validModes = {'none', 'euclidean', 'geo-aware'};
            type = lower(type);
            if ~ismember(type, validModes)
                error('fth:Config:InvalidAdaptationMode', ...
                    'Unknown adaptation mode ''%s''. Valid modes: %s', ...
                    type, strjoin(validModes, ', '));
            end
            obj.controller.adaptation = type;
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
            nCoriolisForm = numel(obj.getCoriolisFormBatchEntries());
            batchCount = gainBatchCount * obj.getTrajectoryBatchCount() * nCoriolisForm;
        end

        function cfgs = expandBatchConfigs(obj, parentResultsDir)
            %EXPANDBATCHCONFIGS Expand a batched config into per-run configs.
            if nargin < 2
                parentResultsDir = '';
            end
            gainBatchCount = obj.getSharedGainBatchCount();
            [trajNames, trajHover, hasHoverOverride] = obj.getTrajectoryBatchEntries();
            forms = obj.getCoriolisFormBatchEntries();
            nCoriolisForm = numel(forms);
            batchCount = gainBatchCount * numel(trajNames) * nCoriolisForm;
            cfgs = cell(batchCount, 1);
            cfgIndex = 1;
            for trajIdx = 1:numel(trajNames)
                currentTrajName = trajNames(trajIdx);
                for formIdx = 1:nCoriolisForm
                    for gainIdx = 1:gainBatchCount
                        cfgCopy = obj.copy();
                        if hasHoverOverride
                            cfgCopy.applyTrajectoryDefinition(currentTrajName{1}, trajHover(trajIdx));
                            cfgCopy.traj.batch = struct( ...
                                'names', {currentTrajName}, ...
                                'goToHoverBeforePathStarts', trajHover(trajIdx));
                        else
                            cfgCopy.applyTrajectoryDefinition(currentTrajName{1});
                            cfgCopy.traj.batch = struct( ...
                                'names', {currentTrajName});
                        end
                        cfgCopy.controller.Kp = obj.selectGainRow(obj.controller.Kp, gainIdx);
                        cfgCopy.controller.Kd = obj.selectGainRow(obj.controller.Kd, gainIdx);
                        if isfield(obj.controller, 'Gamma') && ~isempty(obj.controller.Gamma)
                            cfgCopy.controller.Gamma = obj.selectGainRow(obj.controller.Gamma, gainIdx);
                        end
                        cfgCopy.controller.coriolisFactorization = forms{formIdx};
                        if ~isempty(parentResultsDir)
                            % Build leaf folder: form name when batching forms,
                            % run_NNN when batching gains, or both combined.
                            if nCoriolisForm > 1 && gainBatchCount > 1
                                leafFolder = fullfile(forms{formIdx}, sprintf('run_%03d', gainIdx));
                            elseif nCoriolisForm > 1
                                leafFolder = forms{formIdx};
                            else
                                leafFolder = sprintf('run_%03d', gainIdx);
                            end
                            if numel(trajNames) > 1
                                trajFolder = obj.getTrajectoryFolderName(currentTrajName{1}, trajIdx);
                                cfgCopy.sim.resultsDirOverride = fullfile(parentResultsDir, trajFolder, leafFolder);
                            else
                                cfgCopy.sim.resultsDirOverride = fullfile(parentResultsDir, leafFolder);
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
            %     .type   - adaptation mode: 'none','euclidean','geo-aware'
            %     .Gamma  - 10x1 adaptive gains
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
            %     .mass     - payload mass [kg]
            %     .CoG      - 3x1 payload CoG offset [m]
            %     .dropTime - drop time [s]
            %
            %   Output:
            %     obj - Config instance (for chaining).
            if isfield(opts, 'mass'),     obj.payload.mass     = opts.mass;     end
            if isfield(opts, 'CoG'),      obj.payload.CoG      = opts.CoG(:);   end
            if isfield(opts, 'dropTime'), obj.payload.dropTime = opts.dropTime; end
        end

        function obj = useSimOptions(obj, opts)
            %USESIMOPTIONS Apply simulation timing settings from a struct.
            %   Recognised fields:
            %     .dt           - simulation integration timestep [s]
            %     .duration     - total run time [s] (required together with .dt)
            %     .controlDt    - controller update period [s]
            %     .adaptationDt - adaptation update period [s]
            %
            %   Output:
            %     obj - Config instance (for chaining).
            if isfield(opts, 'dt') && isfield(opts, 'duration')
                obj.setSimParams(opts.dt, opts.duration);
            end
            if isfield(opts, 'controlDt'),    obj.setControlParams(opts.controlDt);       end
            if isfield(opts, 'adaptationDt'), obj.setAdaptationParams(opts.adaptationDt); end
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
            obj.vehicle.I_params = [0.04092, 0.04017, 0.06921, 5.656e-5, -6.494e-5, 1.313e-5];
            % Compute I6 immediately using utility
            obj.vehicle.I6 = fth.utils.getGeneralizedInertia(obj.vehicle.m, obj.vehicle.I_params, obj.vehicle.CoG);
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

        function validateTrajectoryBatch(obj)
            %VALIDATETRAJECTORYBATCH Validate trajectory batch configuration.
            [trajNames, trajHover, hasHoverOverride] = obj.getTrajectoryBatchEntries();
            if isempty(trajNames)
                error('Config:InvalidTrajectoryBatch', 'At least one trajectory must be configured.');
            end
            if hasHoverOverride && numel(trajHover) ~= numel(trajNames)
                error('Config:InvalidTrajectoryHover', ...
                    'Trajectory goToHoverBeforePathStarts must be a scalar or match the number of trajectories.');
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

        function trajHover = normalizeTrajectoryHover(~, goToHoverBeforePathStarts, count)
            %NORMALIZETRAJECTORYHOVER Normalize trajectory hover input.
            if isscalar(goToHoverBeforePathStarts)
                trajHover = repmat(logical(goToHoverBeforePathStarts), 1, count);
                return;
            end
            trajHover = logical(goToHoverBeforePathStarts(:)).';
            if numel(trajHover) ~= count
                error('Config:InvalidTrajectoryHover', ...
                    'Trajectory goToHoverBeforePathStarts must be a scalar or match the number of trajectories.');
            end
        end

        function folderName = getTrajectoryFolderName(~, name, index)
            %GETTRAJECTORYFOLDERNAME Build a compact trajectory folder name.
            shortName = fth.io.NamingUtils.trajectoryLabel(name);
            folderName = sprintf('t%02d_%s', index, shortName);
        end
    end
end
