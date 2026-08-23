function startup()
%STARTUP Add project paths
    repoRoot = fileparts(mfilename('fullpath'));
    addpath(repoRoot);
    addpath(fullfile(repoRoot, 'src'));
    addpath(genpath(fullfile(repoRoot, 'trajectories')));
    addpath(fullfile(repoRoot, 'run'));
    fprintf('[startup] Paths added. Ready.\n');
end
