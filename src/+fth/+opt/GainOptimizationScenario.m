classdef GainOptimizationScenario
    %GAINOPTIMIZATIONSCENARIO Catalog, select, and build tuning scenarios.
    methods (Static)
        function scenarios = catalog()
            base = { ...
                'nominal-basic', 'Nominal / basic', 'none', 'basic';
                'nominal-consistent', 'Nominal / consistent', 'none', 'consistent';
                'adaptive-basic-euclidean', 'Adaptive / basic / Euclidean', 'euclidean', 'basic';
                'adaptive-basic-bregman', 'Adaptive / basic / Bregman', 'bregman', 'basic';
                'adaptive-consistent-euclidean', 'Adaptive / consistent / Euclidean', 'euclidean', 'consistent';
                'adaptive-consistent-bregman', 'Adaptive / consistent / Bregman', 'bregman', 'consistent'};
            scenarios = repmat(struct('id', '', 'label', '', 'adaptation', '', ...
                'coriolisForm', '', 'paramInit', '', 'withPayload', false, ...
                'clearCache', []), 1, 12);
            % Empty clearCache values inherit the runner-level policy.
            cursor = 0;
            for i = 1:size(base, 1)
                for withPayload = [false true]
                    cursor = cursor + 1;
                    suffix = 'no-payload';
                    paramInit = 'vehicle-slight-dev';
                    if withPayload, suffix = 'payload-drop'; paramInit = 'mid-vehicle-payload'; end
                    scenarios(cursor).id = [base{i,1} '-' suffix];
                    scenarios(cursor).label = [base{i,2} ' / ' strrep(suffix, '-', ' ')];
                    scenarios(cursor).adaptation = base{i,3};
                    scenarios(cursor).coriolisForm = base{i,4};
                    scenarios(cursor).paramInit = paramInit;
                    scenarios(cursor).withPayload = withPayload;
                end
            end
        end

        function scenarios = select(catalog, requested)
            if (ischar(requested) || (isstring(requested) && isscalar(requested))) && strcmpi(char(requested), 'all')
                scenarios = catalog; return;
            end
            if ischar(requested) || (isstring(requested) && isscalar(requested))
                requested = {char(requested)};
            elseif isstring(requested)
                requested = cellstr(requested(:));
            end
            if ~iscellstr(requested) || isempty(requested)
                error('fth:GainOptimizer:InvalidScenarios', ...
                    'Scenarios must be ''all'', an ID, or a cell array of IDs.');
            end
            ids = {catalog.id};
            [found, locations] = ismember(requested, ids);
            if ~all(found)
                error('fth:GainOptimizer:UnknownScenario', ...
                    'Unknown scenario ID(s): %s', strjoin(requested(~found), ', '));
            end
            scenarios = catalog(locations);
        end

        function cfg = build(scenario, duration, replayId)
            if nargin < 3 || isempty(replayId)
                replayId = 'ellipse_01_auto';
            end
            cfg = fth.sim.Config();
            cfg.useSimOptions(struct('dt', 0.005, 'duration', duration, 'controlDt', 0.01, ...
                'adaptationDt', 0.005, 'enableSafety', false, 'parallelRuns', false, ...
                'scriptName', ['gain_tuning_' scenario.id]));
            cfg.useTrajectoryOptions(struct('name', 'replay', 'replay', struct('id', replayId)));
            cfg.useControllerOptions(struct('potential', 'inertia-gain', 'paramInit', scenario.paramInit, ...
                'coriolisForm', scenario.coriolisForm, 'lambda', [0.5; 0.5; 0.5; 0.2; 0.2; 0.2]));
            cfg.setAdaptation(scenario.adaptation);
            if ~strcmp(scenario.adaptation, 'none')
                cfg.useAdaptationOptions(struct('type', scenario.adaptation, 'Gamma', 1e-3, ...
                    'useBackTracking', true));
            end
            if scenario.withPayload
                cfg.usePayloadOptions(struct('mass', 1.5, 'position', [0.115; 0.05; -0.25], ...
                    'dims', [0.20, 0.20, 0.10], 'dropTime', 2 * duration / 3));
            end
            cfg.useVizOptions(struct('enable', false, 'liveSummary', false));
        end
    end
end
