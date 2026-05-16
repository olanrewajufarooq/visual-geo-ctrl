classdef SimRunnerUtils
    %SIMRUNNERUTILS Static utility helpers for fth.sim.SimRunner.
    %   All methods are static — no instance needed.

    methods (Static)

        % ------------------------------------------------------------------
        % Struct builders
        % ------------------------------------------------------------------

        function actual = buildActual(H, V)
            %BUILDACTUAL Package actual state for logging.
            pos = H(1:3,4);
            rpy = fth.se3.rotm2rpy(H(1:3,1:3)).';
            actual = struct('pos', pos', 'rpy', rpy, 'linVel', V(4:6)', 'angVel', V(1:3)');
        end

        function desired = buildDesired(Hd, Vd, Ad)
            %BUILDDESIRED Package desired state for logging.
            pos = Hd(1:3,4);
            rpy = fth.se3.rotm2rpy(Hd(1:3,1:3)).';
            Ad = Ad(:);
            desired = struct('pos', pos', 'rpy', rpy, 'linVel', Vd(4:6)', 'angVel', Vd(1:3)', 'acc6', Ad.');
        end

        function cmd = buildCmd(W_cmd)
            %BUILDCMD Package command wrench for logging.
            cmd = struct('wrenchF', W_cmd(4:6)', 'wrenchT', W_cmd(1:3)');
        end

        % ------------------------------------------------------------------
        % Estimation init
        % ------------------------------------------------------------------

        function pi = packEstimatePi(m, Iparams, CoG)
            %PACKESTIMATEPI Convert physical parameters into pi form.
            %   Iparams ordering: [Ixx; Iyy; Izz; Ixy; Ixz; Iyz]
            %   pi ordering: [m; hx; hy; hz; Ixx; Iyy; Izz; Ixy; Ixz; Iyz]
            pi = [m; m * CoG(:); Iparams(:)];
        end

        function [pi0, initLabel] = resolveEstimateInitializationTheta( ...
                initCfg, m_base, I_base, cog_base, m_true, I_true, cog_true)
            %RESOLVEESTIMATEINITIALIZATIONTHETA Build initial adaptive pi.
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
                    pi0 = fth.sim.SimRunnerUtils.packEstimatePi(m_base, I_base, cog_base);
                    initLabel = 'VEHICLE';
                case 'vehicle-plus-payload'
                    pi0 = fth.sim.SimRunnerUtils.packEstimatePi(m_true, I_true, cog_true);
                    initLabel = 'VEHICLE-PLUS-PAYLOAD';
                case 'mid-vehicle-payload'
                    if isempty(spec)
                        pi0 = fth.sim.SimRunnerUtils.buildDefaultFixedEstimatePi(m_base, I_base, cog_base, m_true, I_true, cog_true);
                    else
                        validateattributes(spec, {'numeric'}, {'vector', 'numel', 10});
                        pi0 = spec(:);
                    end
                    initLabel = 'MID-VEHICLE-PAYLOAD';
                case 'vehicle-plus-payload-higher'
                    if isempty(spec)
                        pi0 = fth.sim.SimRunnerUtils.buildDefaultFixedHigherEstimatePi(m_base, I_base, cog_base, m_true, I_true, cog_true);
                    else
                        validateattributes(spec, {'numeric'}, {'vector', 'numel', 10});
                        pi0 = spec(:);
                    end
                    initLabel = 'VEHICLE-PLUS-PAYLOAD-HIGHER';
                case 'vehicle-slight-dev'
                    pi0 = fth.sim.SimRunnerUtils.buildVehicleSlightDevEstimatePi(m_base, I_base, cog_base, spec);
                    initLabel = 'VEHICLE-SLIGHT-DEV';
                case 'custom'
                    validateattributes(spec, {'numeric'}, {'vector', 'numel', 10});
                    pi0 = spec(:);
                    initLabel = 'CUSTOM';
                case 'random'
                    pi0 = fth.sim.SimRunnerUtils.buildRandomEstimatePi(m_true, I_true, cog_true, spec);
                    initLabel = 'RANDOM';
                otherwise
                    error('SimRunner:InvalidEstimateInitializationMode', ...
                        'Unknown estimate initialization mode: %s', mode);
            end
        end

        function pi = buildRandomEstimatePi(m_true, I_true, cog_true, spec)
            %BUILDRANDOMESTIMATEPI Build a deterministic random initial pi.
            seed = 1729;
            if nargin >= 4 && ~isempty(spec)
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

            I_rand   = I_true(:) .* inertiaScale;
            m_rand   = max(1e-6, m_true * massScale);
            cog_rand = cog_true(:) + cogDelta;
            pi = fth.sim.SimRunnerUtils.packEstimatePi(m_rand, I_rand, cog_rand);
        end

        function pi = buildDefaultFixedEstimatePi(m_nom, I_nom, cog_nom, m_true, I_true, cog_true)
            %BUILDDEFAULTFIXEDESTIMATEPI Build the repo default fixed pi.
            %   Alpha weights in pi ordering: [m, h1, h2, h3, I1, I2, I3, I4, I5, I6]
            alpha = [0.58; 0.47; 0.53; 0.50; 0.40; 0.60; 0.45; 0.55; 0.42; 0.50];
            pi_nom  = fth.sim.SimRunnerUtils.packEstimatePi(m_nom,  I_nom,  cog_nom);
            pi_true = fth.sim.SimRunnerUtils.packEstimatePi(m_true, I_true, cog_true);
            pi = pi_nom + alpha .* (pi_true - pi_nom);
        end

        function pi = buildDefaultFixedHigherEstimatePi(m_nom, I_nom, cog_nom, m_true, I_true, cog_true)
            %BUILDDEFAULTFIXEDHIGHERESTIMATEPI Build the repo default fixed-higher pi.
            %   Alpha weights in pi ordering: [m, h1, h2, h3, I1, I2, I3, I4, I5, I6]
            %   Note: unlike buildDefaultFixedEstimatePi, this formula extrapolates
            %   *beyond* pi_true (initial estimate > true value), intentionally
            %   testing adaptation convergence from an overestimate.
            alpha = [0.32; 0.22; 0.28; 0.25; 0.15; 0.35; 0.20; 0.30; 0.18; 0.25];
            pi_nom  = fth.sim.SimRunnerUtils.packEstimatePi(m_nom,  I_nom,  cog_nom);
            pi_true = fth.sim.SimRunnerUtils.packEstimatePi(m_true, I_true, cog_true);
            pi = pi_true + alpha .* (pi_true - pi_nom);
        end

        function pi = buildVehicleSlightDevEstimatePi(m_base, I_base, cog_base, spec)
            %BUILDVEHICLESLIGHTDEVESTIMATEPI Build a slightly perturbed vehicle pi.
            %   Default: +5% mass, +5% diagonal inertia terms, +CoG offset.
            %   If spec is a 10×1 numeric vector it is used directly as pi.
            if nargin >= 4 && ~isempty(spec)
                validateattributes(spec, {'numeric'}, {'vector', 'numel', 10});
                pi = spec(:);
                return;
            end
            m_dev   = m_base * 1.15;
            I_dev   = I_base(:) .* [1.05; 1.04; 1.05; 1.075; 1.05; 1.025] * 1.15;
            cog_dev = (cog_base(:) + [0.05; 0.025; 0.075]) * 1.15;
            pi      = fth.sim.SimRunnerUtils.packEstimatePi(m_dev, I_dev, cog_dev);
        end

        % ------------------------------------------------------------------
        % Physics
        % ------------------------------------------------------------------

        function [m_total, Iparams_total, cog_total] = addPayload(m_base, Iparams_base, cog_base, m_payload, cog_payload)
            %ADDPAYLOAD Combine payload mass/CoG with base parameters (private).
            m_total   = m_base + m_payload;
            cog_total = (m_base * cog_base(:) + m_payload * cog_payload(:)) / m_total;
            J_base    = fth.se3.rotInertiaVec2Mat(Iparams_base(:));
            r         = cog_payload(:);
            J_payload = m_payload * (dot(r,r) * eye(3) - r * r.');
            J_total   = J_base + J_payload;
            Iparams_total = [J_total(1,1), J_total(2,2), J_total(3,3), ...
                             J_total(1,2), J_total(1,3), J_total(2,3)];
        end

        function out = cleanNearZero(in, tol)
            %CLEANNEARZERO Replace values near zero with exact zero (private).
            if nargin < 2
                tol = 1e-12;
            end

            if isstruct(in)
                out = in;
                fields = fieldnames(in);
                for i = 1:numel(fields)
                    out.(fields{i}) = fth.sim.SimRunnerUtils.cleanNearZero(in.(fields{i}), tol);
                end
            elseif isnumeric(in)
                out = in;
                nearZero = abs(in) < tol;
                out(nearZero) = 0;
            else
                out = in;
            end
        end

        % ------------------------------------------------------------------
        % I/O
        % ------------------------------------------------------------------

        function persistCurrentRun(resultsDir, lastLogs, lastMetrics, lastEst, lastRunInfo, cfg)
            %PERSISTCURRENTRUN Save finalized run data to sim_data.mat.
            fth.io.ResultsManager.persistRun(resultsDir, lastLogs, lastMetrics, lastEst, lastRunInfo, cfg);
        end

        function persistMetricsFile(resultsDir, lastMetrics, lastRunInfo, cfg)
            %PERSISTMETRICSFILE Save lightweight metrics.txt for every run.
            fth.io.ResultsManager.writeMetricsFile(resultsDir, lastMetrics, lastRunInfo, cfg);
        end

        function plotCurrentRun(resultsDir, lastLogs, lastEst, lastRunInfo, cfg, plotType, displayPlots)
            %PLOTCURRENTRUN Plot finalized in-memory run data and save PNGs.
            fth.io.ResultsManager.plotRunData(resultsDir, lastLogs, lastEst, lastRunInfo, cfg, plotType, displayPlots);
        end

        function plotSavedRun(resultsDir, plotType, displayPlots)
            %PLOTSAVEDRUN Generate plots for one saved run directory.
            fth.io.ResultsManager.plotSavedRun(resultsDir, plotType, displayPlots);
        end

        % ------------------------------------------------------------------
        % Setup helpers
        % ------------------------------------------------------------------

        function ctrl = createController(cfg)
            %CREATECONTROLLER Instantiate the configured controller.
            ctrl = fth.ctrl.ControllerFactory.create(cfg);
        end

        function value = getPayloadField(payload, fieldName, defaultValue)
            %GETPAYLOADFIELD Read payload config with fallback.
            value = defaultValue;
            if isstruct(payload) && isfield(payload, fieldName)
                candidate = payload.(fieldName);
                if ~isempty(candidate)
                    value = candidate;
                end
            end
        end

        function root = repoRoot()
            %REPOROOT Return repository root path.
            root = fth.io.ResultsManager.repoRoot();
        end

        % ------------------------------------------------------------------
        % Input parsing
        % ------------------------------------------------------------------

        function tf = isPlotSpecifier(value)
            %ISPLOTSPECIFIER Return true for a plotType string argument.
            tf = (ischar(value) || (isstring(value) && isscalar(value))) ...
                && any(strcmpi(string(value), ["none", "summary", "all"]));
        end

        function [isAdaptive, payloadMass, payloadCoG, payloadDropTime, ...
                plotType, displayPlots, saveSimData] = parseRunInputs(cfg, payload, varargin)
            %PARSERUNINPUTS Parse run inputs plus post-run options.
            if isfield(cfg.controller, 'adaptation')
                isAdaptive = fth.sim.Config.isAdaptationEnabled(cfg.controller.adaptation);
            else
                isAdaptive = false;
            end
            payloadMass = fth.sim.SimRunnerUtils.getPayloadField(payload, 'mass', 0);
            payloadCoG = fth.sim.SimRunnerUtils.getPayloadField(payload, 'CoG', [0;0;0]);
            payloadDropTime = fth.sim.SimRunnerUtils.getPayloadField(payload, 'dropTime', inf);
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

            if ~isempty(args) && ~fth.sim.SimRunnerUtils.isPlotSpecifier(args{1})
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

    end
end
