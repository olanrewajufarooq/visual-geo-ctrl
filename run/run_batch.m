% RUN_BATCH Execute, save, and plot the six payload-drop comparison variants.
%
% Every controller begins with exact loaded parameters. The fixed payload is
% released at t = 10 s, and independent cases use parallel workers by default.

%% User settings

replayId = 'lemniscate_01_auto';

duration = 30;

useParallel = true;

inplaceSave = logical(true); % true: use results/inplace; false: create results/<timestamp>.

generateFigures = true;

showFigures = false;

saveReplay3D = logical(false); % Save replay3d.mp4 beside every successful run.mat.

%% Build the paper comparison matrix

startup;

variants = {'nominal',   'c1'; ...
            'nominal',   'c2'; ...
            'euclidean', 'c1'; ...
            'euclidean', 'c2'; ...
            'bregman',   'c1'; ...
            'bregman',   'c2'};

scenarios = cell(size(variants,1), 1);

for k = 1:size(variants,1)
    scenarios{k} = agc.sim.defaultScenario(replayId, variants{k,1}, variants{k,2}, duration);
end

%% Run scenarios in parallel and save their results

batch = agc.batch.runBatch(scenarios, struct('parallel', useParallel));

root = agc.io.repositoryRoot();

resultsRoot = fullfile(root, 'results');

if inplaceSave
    suiteDirectory = fullfile(resultsRoot, 'inplace');

    % saveBatchSuite overwrites only successful <mode>_<coriolis>/run.mat
    % members; all unrelated in-place result folders remain untouched.
else
    stamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));

    suiteDirectory = fullfile(resultsRoot, stamp);
end

saved = agc.io.saveBatchSuite(suiteDirectory, scenarios, batch);

%% Report the successful and failed variants

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

%% Optionally export one 3-D replay video for every successful variant

if saveReplay3D
    for k = 1:numel(saved)
        if ~saved(k).saved
            continue;
        end

        replayFile = fullfile(saved(k).directory, 'replay3d.mp4');

        try
            agc.viz.replay3D(batch.runs{k}, exportFile=replayFile);

            close(gcf);

            fprintf('Saved 3-D replay: %s\n', replayFile);
        catch ME
            warning('run_batch:Replay3D', '%s replay export failed: %s', ...
                saved(k).name, ME.message);
        end
    end
end

%% Export static paper figures from the saved suite

if generateFigures && batch.successCount > 0
    output = agc.viz.paperFigures(suiteDirectory, ...
        struct('visible', showFigures, 'exportComparisons', ~inplaceSave));

    fprintf('Saved %d figure pairs from %d successful simulations.\n', ...
        numel(output), batch.successCount);
elseif generateFigures
    warning('run_batch:NoSuccessfulRuns', 'Skipping figures because every simulation failed.');
end
