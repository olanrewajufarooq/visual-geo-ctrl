%PROCESS_TRAJECTORIES Build canonical replay artifacts for downloaded flights.
% Run once from MATLAB after the trajectory CSVs have been downloaded. The
% generated MAT artifacts are committed under trajectories/processed/.

processingMethod = 'wnoj';  % Choose 'wnoj' or 'poly'.
clearCache = false;

fprintf('[replay] Starting trajectory preprocessing.\n');
summary = ReplayProcessor.processAll(processingMethod, clearCache);
fprintf('Processed %d replay trajectories.\n', summary.processedCount);
