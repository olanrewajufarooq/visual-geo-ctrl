% OPTIMIZE_GAINS Tune and promote all gains for one paper scenario.
% The search includes KR(3), Kxi(3), Lambda(6), kd, ks, alpha, and the
% mode-specific adaptation gain: gammaE(10) or gammaB(1). Edit settings,
% then press Run. Each winner is written to config/optimized_gains.m.

%% User settings

replayId = 'lemniscate_01_auto';
% Use '' for every option, one char/string for one option, or a cell array.
% Examples: mode = ''; coriolis = ''; or mode = {'bregman','euclidean'};
mode = '';
coriolis = '';
duration = 30;
useParallel = true;
swarmSize = 50;
maxIterations = 50;
functionTolerance = 1e-3;
maxStallIterations = 10;
promoteBest = true;

%% Expand the requested scenario batch

startup;
variants = agc.opt.expandScenarioSelection(mode, coriolis);
weights = struct('position', 1, 'attitude', 1, 'effort', 1e-3, 'failure', 1e6);
root = agc.io.repositoryRoot();
stamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));

%% Optimize each selected scenario and save/promote its independent winner

% Each particleswarm run parallelizes its candidate evaluations internally.
% Keeping this outer loop serial prevents nested parallel pools.
for variantIndex = 1:size(variants, 1)
    selectedMode = variants{variantIndex, 1};
    selectedCoriolis = variants{variantIndex, 2};
    fprintf('Optimizing %s/%s (%d of %d)\n', selectedMode, selectedCoriolis, ...
        variantIndex, size(variants, 1));

    base = agc.sim.defaultScenario(replayId, selectedMode, selectedCoriolis, duration, 'manual');
    optimizedBase = agc.sim.defaultScenario(replayId, selectedMode, selectedCoriolis, duration, 'optimized');
    [lowerBound, upperBound] = agc.opt.gainBounds(selectedMode);
    % Always evaluate known feasible baselines before exploring random points.
    % Legacy promoted values outside revised bounds are projected by optimize.
    initialPoints = [agc.opt.encodeScenarioGains(base); ...
        agc.opt.encodeScenarioGains(optimizedBase)];
    initialPoints = unique(initialPoints, 'rows', 'stable');
    result = agc.opt.optimize({base}, @agc.opt.applyScenarioGains, lowerBound, upperBound, ...
        struct('weights', weights, 'swarmSize', swarmSize, 'maxIterations', maxIterations, ...
        'functionTolerance', functionTolerance, 'maxStallIterations', maxStallIterations, ...
        'parallel', useParallel, 'initialPoints', initialPoints));

    bestScenario = agc.opt.applyScenarioGains(result.candidate, {base});
    bestController = bestScenario{1}.controller;
    bestGains = struct('KRdiag', diag(bestController.KR).', ...
        'Kxidiag', diag(bestController.Kxi).', ...
        'LambdaDiag', diag(bestController.Lambda).', ...
        'kd', bestController.kd, 'ks', bestController.ks, 'alpha', bestController.alpha, ...
        'gammaE', bestController.gammaE(:), 'gammaB', bestController.gammaB);
    bestGains = agc.opt.roundGains(bestGains, 4);

    resultDirectory = fullfile(root, 'results', 'optimization', stamp, ...
        sprintf('%s_%s', selectedMode, selectedCoriolis));
    if ~isfolder(resultDirectory), mkdir(resultDirectory); end
    save(fullfile(resultDirectory, 'optimization.mat'), 'result', 'bestGains', 'bestScenario', '-v7.3');

    if promoteBest
        agc.io.promoteOptimizedGains(selectedMode, selectedCoriolis, bestGains, ...
            struct('cost', result.cost, 'artifact', resultDirectory));
        clear optimized_gains
    end

    fprintf('Best cost for %s/%s: %.6g\n', selectedMode, selectedCoriolis, result.cost);
    fprintf('Saved optimization artifact: %s\n', resultDirectory);
end
