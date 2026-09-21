% RUN_THEORY_SUITE Run all paper variants in the 10 s payload-drop experiment.
%
% Every controller begins with the exact loaded model; only adaptive estimates
% can respond after the 0.75 kg offset payload is released from the bare UAV.

%% User settings

replayId = 'lemniscate_01_auto';

duration = 30;

useParallel = true;

inplaceSave = logical(true); % true: use results/inplace; false: create results/<timestamp>.

generateFigures = true; % Export plots for every completed run or finite prefix.

showFigures = false;

saveReplay3D = true; % Save replay3d.mp4 beside every persisted run.mat.

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

    % saveBatchSuite overwrites only selected <mode>_<coriolis>/run.mat
    % members; all unrelated in-place result folders remain untouched.
else
    stamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));

    suiteDirectory = fullfile(resultsRoot, stamp);
end

saved = agc.io.saveBatchSuite(suiteDirectory, scenarios, batch);

%% Report every requested variant and retain finite runtime-failure prefixes

for k = 1:numel(saved)
    if saved(k).successful
        metrics = batch.metrics{k};

        fprintf('%s/%s: position RMSE %.4g m, attitude RMSE %.4g rad\n', ...
            variants{k,1}, variants{k,2}, metrics.positionRMSE, metrics.attitudeRMSE);
    elseif saved(k).saved
        warning('run_theory_suite:PartialVariant', ...
            '%s/%s failed after %.4g s; saved its finite prefix: %s', ...
            variants{k,1}, variants{k,2}, batch.failures{k}.time, batch.failures{k}.message);
    else
        warning('run_theory_suite:FailedVariant', '%s/%s failed: %s', ...
            variants{k,1}, variants{k,2}, batch.failures{k}.message);
    end
end

fprintf('Saved result suite: %s\n', suiteDirectory);

%% Optionally export one 3-D replay video for every persisted variant

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

%% Export figures for every persisted run and compatible comparisons

if generateFigures && any([saved.saved])
    output = agc.viz.paperFigures(suiteDirectory, ...
        struct('visible', showFigures, 'exportComparisons', ~inplaceSave));

    fprintf('Saved %d figure pairs from %d persisted simulations.\n', ...
        numel(output), sum([saved.saved]));
elseif generateFigures
    warning('run_theory_suite:NoSavedRuns', ...
        'Skipping figures because no requested simulation logged a finite prefix.');
end
