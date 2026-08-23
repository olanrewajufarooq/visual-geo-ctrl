%PROCESS_TRAJECTORIES Build canonical replay artifacts for downloaded flights.
% Run once from MATLAB after the trajectory CSVs have been downloaded.

scriptDir = fileparts(mfilename('fullpath'));
repoRoot = fileparts(scriptDir);
addpath(genpath(fullfile(repoRoot, 'src')));
addpath(genpath(scriptDir));

processedRoot = fullfile(scriptDir, 'processed');
manifestPath = fullfile(processedRoot, 'manifest.json');
summary = ReplayProcessor.processAll(processedRoot, manifestPath);
fprintf('Processed %d replay trajectories.\n', summary.processedCount);
