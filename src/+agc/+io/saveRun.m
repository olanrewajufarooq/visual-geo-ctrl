function saveRun(resultDirectory, scenario, run, metrics)
%SAVERUN Persist the immutable scenario and its result in one MAT file.
if ~isfolder(resultDirectory), mkdir(resultDirectory); end
save(fullfile(resultDirectory, 'run.mat'), 'scenario', 'run', 'metrics', '-v7.3');
end
