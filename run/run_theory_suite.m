% RUN_THEORY_SUITE Run all paper controller variants on one replay artifact.
% Edit these settings, then press Run.
replayId = 'lemniscate_01_auto';
duration = 10;
useParallel = true;
startup;
variants = { ...
    'nominal', 'c1'; 'nominal', 'c2'; ...
    'euclidean', 'c1'; 'euclidean', 'c2'; ...
    'bregman', 'c1'; 'bregman', 'c2'};
root = agc.io.repositoryRoot(); stamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
scenarios = cell(size(variants, 1), 1);
for k = 1:size(variants, 1)
    scenarios{k} = agc.sim.defaultScenario(replayId, variants{k,1}, variants{k,2}, duration);
end
batch = agc.batch.runBatch(scenarios, struct('parallel', useParallel));
for k = 1:size(variants, 1)
    if ~isempty(batch.failures{k})
        warning('run_theory_suite:FailedVariant', '%s/%s failed: %s', ...
            variants{k,1}, variants{k,2}, batch.failures{k}.message);
        continue;
    end
    run = batch.runs{k}; metrics = batch.metrics{k}; scenario = scenarios{k};
    resultDirectory = fullfile(root, 'results', 'v2', stamp, ...
        sprintf('%s_%s', variants{k,1}, variants{k,2}));
    agc.io.saveRun(resultDirectory, scenario, run, metrics);
    fprintf('%s/%s: position RMSE %.4g m, attitude RMSE %.4g rad\n', ...
        variants{k,1}, variants{k,2}, metrics.positionRMSE, metrics.attitudeRMSE);
end
