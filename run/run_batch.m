% RUN_BATCH Execute the paper variant matrix through the shared batch runner.
% Edit these settings, then press Run.
replayId = 'lemniscate_01_auto';
duration = 10;
useParallel = true;
startup;
variants = {'nominal','c1'; 'nominal','c2'; 'euclidean','c1'; ...
            'euclidean','c2'; 'bregman','c1'; 'bregman','c2'};
scenarios = cell(size(variants,1), 1);
for k = 1:size(variants,1)
    scenarios{k} = agc.sim.defaultScenario(replayId, variants{k,1}, variants{k,2}, duration);
end
batch = agc.batch.runBatch(scenarios, struct('parallel', useParallel));
disp(batch);
