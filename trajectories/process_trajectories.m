%PROCESS_TRAJECTORIES Build canonical replay artifacts for downloaded flights.
% Run once from MATLAB after the trajectory CSVs have been downloaded. The
% generated MAT artifacts are committed under trajectories/processed/.

processingMethod = 'wnoj';  % Choose 'wnoj' or 'poly'.
if ~ismember(processingMethod, {'wnoj', 'poly'})
    error('fth:Replay:InvalidProcessingMethod', ...
        'processingMethod must be ''wnoj'' or ''poly'' (got ''%s'').', ...
        processingMethod);
end

scriptDir = fileparts(mfilename('fullpath'));
repoRoot = fileparts(scriptDir);
addpath(genpath(fullfile(repoRoot, 'src')));
addpath(genpath(scriptDir));

processedRoot = fullfile(scriptDir, 'processed');
manifestPath = fullfile(processedRoot, 'manifest.json');
fprintf('[replay] Starting trajectory preprocessing.\n');
summary = ReplayProcessor.processAll(processedRoot, manifestPath, [], ...
    processingMethod);
fprintf('Processed %d replay trajectories.\n', summary.processedCount);
