function summary = process_trajectories(rootDir, manifestPath)
%PROCESS_TRAJECTORIES Build processed replay artifacts from raw CSV logs.
    if nargin < 1 || isempty(rootDir)
        rootDir = fullfile(fileparts(mfilename('fullpath')), '..', 'trajectories', 'processed');
    end
    if nargin < 2 || isempty(manifestPath)
        manifestPath = fullfile(rootDir, 'manifest.json');
    end

    summary = fth.traj.ReplayProcessor.processAll(rootDir, manifestPath);
    fprintf('[process_trajectories] Processed %d trajectories using %s\n', ...
        summary.processedCount, summary.manifestPath);
end
