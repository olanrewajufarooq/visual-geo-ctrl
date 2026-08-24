classdef GainEvaluationProblem < handle
    %GAINEVALUATIONPROBLEM Evaluate shared controller gains with replay rollouts.
    %   Bregman uses 19 variables: Kp(6), Kd(6), lambda(6), log10(Gamma).
    %   Euclidean uses 28 variables: Kp(6), Kd(6), lambda(6), log10(Gamma(10)).

    properties (SetAccess = private)
        Mode
        BaseConfigs
        LowerBound
        UpperBound
        BaselineVector
        Normalizers
        BaselineFailures
        Options
    end

    properties (Access = private)
        LastCompletedFraction = 0
    end

    methods
        function obj = GainEvaluationProblem(mode, baseConfigs, options)
            arguments
                mode (1,:) char
                baseConfigs cell
                options struct = struct()
            end
            obj.Mode = lower(mode);
            if ~ismember(obj.Mode, {'none', 'bregman', 'euclidean'})
                error('fth:GainEvaluationProblem:InvalidMode', ...
                    'Mode must be ''none'', ''bregman'', or ''euclidean''.');
            end
            if isempty(baseConfigs)
                error('fth:GainEvaluationProblem:MissingConfigs', ...
                    'At least one scenario configuration is required.');
            end
            obj.BaseConfigs = baseConfigs(:);
            obj.Options = obj.defaultOptions(options);
            nGamma = obj.gammaWidth();
            bounds = obj.Options.bounds;
            obj.validateBounds(bounds);
            obj.LowerBound = [bounds.Kp(1) * ones(1, 6), bounds.Kd(1) * ones(1, 6), ...
                bounds.lambda(1) * ones(1, 6), bounds.gammaLog10(1) * ones(1, nGamma)];
            obj.UpperBound = [bounds.Kp(2) * ones(1, 6), bounds.Kd(2) * ones(1, 6), ...
                bounds.lambda(2) * ones(1, 6), bounds.gammaLog10(2) * ones(1, nGamma)];
            % The replay adaptive baseline needs the same composite-variable
            % coupling used by the stable adaptive run scripts. Lambda remains
            % a tunable parameter, but normalization must start from a usable
            % closed-loop controller rather than the zero-coupling case.
            obj.BaselineVector = [5.5 * ones(1, 6), 2.05 * ones(1, 6), ...
                [0.5, 0.5, 0.5, 0.2, 0.2, 0.2], ...
                log10(1e-3) * ones(1, nGamma)];
            obj.Normalizers = obj.Options.normalizers;
            obj.BaselineFailures = false(numel(obj.BaseConfigs), 1);
        end

        function n = dimension(obj)
            n = 18 + obj.gammaWidth();
        end

        function [cfg, decoded] = decode(obj, x, cfg)
            %DECODE Decode gains and optionally apply them to a Config copy.
            x = double(x(:).');
            if numel(x) ~= obj.dimension()
                error('fth:GainEvaluationProblem:InvalidVector', ...
                    'Expected %d variables, received %d.', obj.dimension(), numel(x));
            end
            decoded = struct('Kp', x(1:6).', 'Kd', x(7:12).', ...
                'lambda', x(13:18).', 'Gamma', 10 .^ x(19:end).');
            if nargin < 3 || isempty(cfg)
                cfg = [];
                return;
            end
            cfg.setKpGains(decoded.Kp);
            cfg.setKdGains(decoded.Kd);
            cfg.setLambda(decoded.lambda);
            cfg.setAdaptiveGains(decoded.Gamma);
        end

        function [cost, breakdown] = evaluate(obj, x)
            %EVALUATE Evaluate all scenarios and return a finite PSO objective.
            x = double(x(:).');
            if numel(x) ~= obj.dimension() || any(x < obj.LowerBound) || any(x > obj.UpperBound)
                cost = obj.failurePenalty(0);
                breakdown = struct('failed', true, 'reason', 'bounds', ...
                    'scenario', [], 'cost', cost);
                return;
            end
            if isempty(obj.Normalizers)
                obj.Normalizers = obj.computeBaselineNormalizers();
            end
            emptyResult = struct('failed', false, 'reason', '', 'cost', inf, ...
                'metrics', struct(), 'effort', NaN, 'elapsed', NaN);
            scenarioResults = repmat(emptyResult, numel(obj.BaseConfigs), 1);
            costs = zeros(numel(obj.BaseConfigs), 1);
            for i = 1:numel(obj.BaseConfigs)
                [costs(i), scenarioResults(i)] = obj.evaluateScenario(x, i);
            end
            cost = mean(costs);
            breakdown = struct('failed', any([scenarioResults.failed]), ...
                'scenario', scenarioResults, 'cost', cost, 'vector', x);
        end

        function [baseline, breakdown] = evaluateBaseline(obj)
            %EVALUATEBASELINE Evaluate baseline gains and establish normalizers.
            obj.Normalizers = obj.computeBaselineNormalizers();
            baseline = obj.Normalizers;
            [~, breakdown] = obj.evaluate(obj.BaselineVector);
            breakdown.baselineFailures = obj.BaselineFailures;
        end
    end

    methods (Access = private)
        function n = gammaWidth(obj)
            if strcmp(obj.Mode, 'none')
                n = 0;
            elseif strcmp(obj.Mode, 'bregman')
                n = 1;
            else
                n = 10;
            end
        end

        function validateBounds(~, bounds)
            names = {'Kp', 'Kd', 'lambda', 'gammaLog10'};
            for i = 1:numel(names)
                name = names{i};
                value = bounds.(name);
                if ~isnumeric(value) || numel(value) ~= 2 || ...
                        any(~isfinite(value)) || value(1) >= value(2)
                    error('fth:GainEvaluationProblem:InvalidBounds', ...
                        'bounds.%s must be a finite increasing two-element vector.', name);
                end
            end
        end

        function options = defaultOptions(~, supplied)
            fallback = struct('position', 1, 'orientation', 1, 'maxPosition', 10, ...
                'forceRms', 1, 'torqueRms', 1, 'effort', 1);
            options = struct( ...
                'runOptions', struct('plotMode', 'none', 'displayPlots', false, 'saveSimData', false), ...
                'positionWeight', 0.40, 'orientationWeight', 0.40, ...
                'maxPositionWeight', 0.10, 'effortWeight', 0.10, ...
                'epsilon', 1e-9, 'maxPositionMagnitude', 1e6, ...
                'maxCommandMagnitude', 1e7, 'normalizers', [], ...
                'strictBaseline', false, 'suppressSimulationOutput', true, ...
                'bounds', struct('Kp', [0.005, 15.0], 'Kd', [0.001, 10.0], ...
                    'lambda', [0.0, 20.0], 'gammaLog10', [-6.0, -1.0]), ...
                'fallbackNormalizers', fallback);
            names = fieldnames(supplied);
            for i = 1:numel(names), options.(names{i}) = supplied.(names{i}); end
        end

        function normalizers = computeBaselineNormalizers(obj)
            normalizers = repmat(struct('position', 1, 'orientation', 1, ...
                'maxPosition', 1, 'forceRms', 1, 'torqueRms', 1, 'effort', 1), ...
                numel(obj.BaseConfigs), 1);
            obj.BaselineFailures = false(numel(obj.BaseConfigs), 1);
            for i = 1:numel(obj.BaseConfigs)
                try
                    [logs, metrics] = obj.runScenario(obj.BaselineVector, i);
                    normalizers(i).position = max(metrics.position.rmse_total, obj.Options.epsilon);
                    normalizers(i).orientation = max(metrics.orientation.rmse_total, obj.Options.epsilon);
                    normalizers(i).maxPosition = max(metrics.position.max_error, obj.Options.epsilon);
                    [forceRms, torqueRms, effort] = obj.controlEffort(logs);
                    normalizers(i).forceRms = max(forceRms, obj.Options.epsilon);
                    normalizers(i).torqueRms = max(torqueRms, obj.Options.epsilon);
                    normalizers(i).effort = max(effort, obj.Options.epsilon);
                catch err
                    if obj.Options.strictBaseline
                        rethrow(err);
                    end
                    obj.BaselineFailures(i) = true;
                    normalizers(i) = obj.Options.fallbackNormalizers;
                end
            end
        end

        function [cost, result] = evaluateScenario(obj, x, index)
            result = struct('failed', false, 'reason', '', 'cost', inf, ...
                'metrics', struct(), 'effort', NaN, 'elapsed', NaN);
            timer = tic;
            try
                [logs, metrics] = obj.runScenario(x, index);
                [~, ~, effort] = obj.controlEffort(logs);
                n = obj.Normalizers(index);
                terms = [metrics.position.rmse_total / n.position, ...
                    metrics.orientation.rmse_total / n.orientation, ...
                    metrics.position.max_error / n.maxPosition, effort / n.effort];
                if any(~isfinite(terms)), error('fth:GainEvaluationProblem:NonFiniteCost', ...
                        'Objective terms are not finite.'); end
                cost = obj.Options.positionWeight * terms(1) + ...
                    obj.Options.orientationWeight * terms(2) + ...
                    obj.Options.maxPositionWeight * terms(3) + ...
                    obj.Options.effortWeight * terms(4);
                result.metrics = metrics;
                result.effort = effort;
            catch err
                cost = obj.failurePenalty(obj.LastCompletedFraction);
                result.failed = true;
                result.reason = err.message;
            end
            result.cost = cost;
            result.elapsed = toc(timer);
        end

        function [logs, metrics] = runScenario(obj, x, index)
            cfg = obj.BaseConfigs{index}.copy();
            [cfg, decoded] = obj.decode(x, cfg); %#ok<ASGLU>
            cfg.setAdaptation(obj.Mode);
            if ~strcmp(obj.Mode, 'none')
                cfg.useAdaptationOptions(struct('type', obj.Mode, 'Gamma', decoded.Gamma, ...
                    'useBackTracking', true));
            end
            % Objective evaluations may run concurrently. Keep their transient
            % simulator artifacts out of the shared results tree and give each
            % evaluation a unique directory.
            cfg.sim.resultsDirOverride = tempname;
            cfg.done();
            sim = fth.sim.SimRunner(cfg);
            if obj.Options.suppressSimulationOutput
                % Keep the optimizer console readable while retaining logs.
                evalc('sim.setup(); sim.run(obj.Options.runOptions);');
            else
                sim.setup();
                sim.run(obj.Options.runOptions);
            end
            logs = sim.getLogs();
            if ~isempty(logs) && isfield(logs, 't') && ~isempty(logs.t)
                obj.LastCompletedFraction = max(0, min(1, logs.t(end) / cfg.sim.duration));
            else
                obj.LastCompletedFraction = 0;
            end
            obj.validateLogs(logs, cfg);
            metrics = fth.core.TrackingMetrics(logs, 'gain-evaluation').computeAll();
        end

        function validateLogs(obj, logs, cfg)
            if isempty(logs) || ~isfield(logs, 't') || isempty(logs.t)
                error('fth:GainEvaluationProblem:MissingLogs', 'Simulation returned no logs.');
            end
            if ~isfield(logs, 'actual') || ~isfield(logs.actual, 'pos') || ...
                    any(~isfinite(logs.actual.pos), 'all') || ...
                    max(abs(logs.actual.pos), [], 'all') > obj.Options.maxPositionMagnitude
                error('fth:GainEvaluationProblem:Diverged', 'Position state diverged.');
            end
            if ~isfield(logs, 'cmd') || ~isfield(logs.cmd, 'wrenchF') || ...
                    ~isfield(logs.cmd, 'wrenchT') || any(~isfinite(logs.cmd.wrenchF), 'all') || ...
                    any(~isfinite(logs.cmd.wrenchT), 'all') || ...
                    max(abs([logs.cmd.wrenchF(:); logs.cmd.wrenchT(:)])) > obj.Options.maxCommandMagnitude
                error('fth:GainEvaluationProblem:InvalidCommands', 'Command logs are invalid.');
            end
            if logs.t(end) < cfg.sim.duration - 2 * cfg.sim.dt
                error('fth:GainEvaluationProblem:EarlyStop', 'Simulation stopped before its duration.');
            end
        end

        function [forceRms, torqueRms, effort] = controlEffort(~, logs)
            t = logs.t(:);
            force = logs.cmd.wrenchF;
            torque = logs.cmd.wrenchT;
            forceRms = sqrt(mean(sum(force.^2, 2)));
            torqueRms = sqrt(mean(sum(torque.^2, 2)));
            forceScale = max(forceRms, 1e-6);
            torqueScale = max(torqueRms, 1e-6);
            signal = sum((force ./ forceScale).^2, 2) + ...
                sum((torque ./ torqueScale).^2, 2);
            effort = trapz(t, signal) / max(t(end) - t(1), 1e-6);
        end

        function penalty = failurePenalty(~, completedFraction)
            penalty = 1e3 * (1 + max(0, min(1, 1 - completedFraction)));
        end
    end
end
