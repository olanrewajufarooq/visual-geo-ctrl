% RUN_THEORY_SUITE Run all paper variants in the 10 s payload-drop experiment.
%
% Every controller begins with the exact loaded model; only adaptive estimates
% can respond after the physical plant becomes the bare UAV.

%% User settings

replayId = 'lemniscate_01_auto';

duration = 30;

useParallel = true;

inplaceSave = logical(true); % true: use results/inplace; false: create results/<timestamp>.

generateFigures = true; % Export plots for every successful variant.

showFigures = false;

saveReplay3D = true; % Save replay3d.mp4 beside every successful run.mat.

startup;

%% Select one or more controller variants

% variants = { ...
%     'nominal', 'c1'; 'nominal', 'c2'; ...
%     'euclidean', 'c1'; 'euclidean', 'c2'; ...
%     'bregman', 'c1'; 'bregman', 'c2'};

variants = {'nominal', 'c2'};

root = agc.io.repositoryRoot();

scenarios = cell(size(variants, 1), 1);

for k = 1:size(variants, 1)
    scenarios{k} = agc.sim.defaultScenario(replayId, variants{k,1}, variants{k,2}, duration);
end

%% Run the selected cases

batch = agc.batch.runBatch(scenarios, struct('parallel', useParallel));

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

%% Report every requested variant and retain all successful runs as a suite

for k = 1:numel(saved)
    if ~saved(k).saved
        warning('run_theory_suite:FailedVariant', '%s/%s failed: %s', ...
            variants{k,1}, variants{k,2}, batch.failures{k}.message);

        continue;
    end

    metrics = batch.metrics{k};

    fprintf('%s/%s: position RMSE %.4g m, attitude RMSE %.4g rad\n', ...
        variants{k,1}, variants{k,2}, metrics.positionRMSE, metrics.attitudeRMSE);
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
            warning('run_theory_suite:Replay3D', '%s replay export failed: %s', ...
                saved(k).name, ME.message);
        end
    end
end

%% Export figures for every successful saved run and compatible comparisons

if generateFigures && batch.successCount > 0
    output = agc.viz.paperFigures(suiteDirectory, ...
        struct('visible', showFigures, 'exportComparisons', ~inplaceSave));

    fprintf('Saved %d figure pairs from %d successful simulations.\n', ...
        numel(output), batch.successCount);
elseif generateFigures
    warning('run_theory_suite:NoSuccessfulRuns', ...
        'Skipping figures because every requested simulation failed.');
end
