% RUN_BATCH Execute, save, and plot the six paper comparison variants.
% Edit these settings, then press Run. Simulations use parallel workers by
% default; figures are generated only after all successful runs are saved.

%% User settings

replayId = 'lemniscate_01_auto';
duration = 30;
useParallel = true;
generateFigures = true;
showFigures = false;

%% Build the paper comparison matrix

startup;
variants = {'nominal','c1'; 'nominal','c2'; 'euclidean','c1'; ...
            'euclidean','c2'; 'bregman','c1'; 'bregman','c2'};
scenarios = cell(size(variants,1), 1);
for k = 1:size(variants,1)
    scenarios{k} = agc.sim.defaultScenario(replayId, variants{k,1}, variants{k,2}, duration);
end

%% Run in parallel, then persist an immutable result suite

batch = agc.batch.runBatch(scenarios, struct('parallel', useParallel));
root = agc.io.repositoryRoot();
stamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
suiteDirectory = fullfile(root, 'results', stamp);
saved = agc.io.saveBatchSuite(suiteDirectory, scenarios, batch);

for k = 1:numel(saved)
    if saved(k).saved
        metrics = batch.metrics{k};
        fprintf('%s: position RMSE %.4g m, attitude RMSE %.4g rad\n', ...
            saved(k).name, metrics.positionRMSE, metrics.attitudeRMSE);
    else
        warning('run_batch:FailedVariant', '%s failed: %s', ...
            saved(k).name, saved(k).failure.message);
    end
end
fprintf('Saved result suite: %s\n', suiteDirectory);

%% Export static paper figures from the saved suite

if generateFigures && batch.successCount == numel(scenarios)
    output = agc.viz.paperFigures(suiteDirectory, struct('visible', showFigures));
    fprintf('Saved %d figure pairs in %s\n', numel(output), output(1).directory);
elseif generateFigures
    warning('run_batch:IncompleteSuite', ...
        'Skipping figures because %d of %d simulations failed.', ...
        numel(scenarios) - batch.successCount, numel(scenarios));
end
