%PROCESS_TRAJECTORIES Build canonical replay artifacts for downloaded flights.

clearCache = false;

trajIds = {'ellipse_01_auto', 'lemniscate_01_auto', 'RATM_01_auto'};
% Set trajIds = {} to process the complete manifest.

fprintf('[replay] Starting trajectory preprocessing.\n');
summary = ReplayProcessor.processAll(clearCache, trajIds);
fprintf('Processed %d replay trajectories.\n', summary.processedCount);
