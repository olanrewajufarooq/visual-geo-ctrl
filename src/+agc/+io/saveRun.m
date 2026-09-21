function saveRun(resultDirectory, scenario, run, metrics, failure)
%SAVERUN Persist one completed run or finite runtime-failure prefix.

if nargin < 5, failure = []; end
if ~isfolder(resultDirectory), mkdir(resultDirectory); end
save(fullfile(resultDirectory, 'run.mat'), 'scenario', 'run', 'metrics', 'failure', '-v7.3');
end
