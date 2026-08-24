classdef GainOptimizationUtils
    %GAINOPTIMIZATIONUTILS Small reusable optimizer helpers.
    methods (Static)
        function opts = defaults()
            opts = struct('scenarios', 'all', 'duration', 25, 'swarmSize', 40, ...
                'maxIterations', 30, 'functionTolerance', 1e-4, ...
                'maxStallIterations', 15, 'randomSeed', 20260824, 'useParallel', true, ...
                'bounds', struct('Kp', [0.005, 15.0], 'Kd', [0.001, 10.0], ...
                    'lambda', [0.0, 20.0], 'gammaLog10', [-6.0, -1.0]));
        end

        function validateEnvironment(useParallel)
            if isempty(ver('globaloptim'))
                error('fth:GainOptimizer:MissingToolbox', 'Global Optimization Toolbox is required for particleswarm.');
            end
            if useParallel && isempty(ver('parallel'))
                error('fth:GainOptimizer:MissingToolbox', 'Parallel Computing Toolbox is required when useParallel is true.');
            end
            if useParallel && isempty(gcp('nocreate')), parpool; end
        end

        function initial = initialSwarm(problem, swarmSize)
            initial = repmat(problem.BaselineVector, swarmSize, 1);
            if swarmSize > 1
                initial(2:end, :) = problem.LowerBound + rand(swarmSize - 1, problem.dimension()) .* ...
                    (problem.UpperBound - problem.LowerBound);
            end
            initial(1, :) = problem.BaselineVector;
        end

        function value = bestParticleValue(optimValues)
            if isfield(optimValues, 'bestfval') && isfinite(optimValues.bestfval)
                value = optimValues.bestfval;
            elseif isfield(optimValues, 'fval') && isfinite(optimValues.fval)
                value = optimValues.fval;
            elseif isfield(optimValues, 'swarmfvals') && ~isempty(optimValues.swarmfvals)
                values = optimValues.swarmfvals(:); values = values(isfinite(values));
                if isempty(values), value = NaN; else, value = min(values); end
            else
                value = NaN;
            end
        end
    end
end
