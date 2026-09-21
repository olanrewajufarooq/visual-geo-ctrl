function payload = loadRun(resultDirectory)
%LOADRUN Load one result written by agc.io.saveRun.
filePath = fullfile(resultDirectory, 'run.mat');
if ~isfile(filePath)
    error('agc:io:loadRun:NotFound', 'No run.mat exists in %s.', resultDirectory);
end
payload = load(filePath);
if ~isfield(payload, 'failure'), payload.failure = []; end
end
