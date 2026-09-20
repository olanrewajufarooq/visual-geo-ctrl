function saved = saveBatchSuite(suiteDirectory, scenarios, batch)
%SAVEBATCHSUITE Persist each successful batch member in a named suite folder.
%
% A suite is directly consumable by agc.viz.paperFigures: every successful
% scenario is stored as <suite>/<mode>_<coriolis>/run.mat. Failures remain in
% the returned manifest so a caller can report them without serializing an
% invalid partial run.

validateattributes(suiteDirectory, {'char', 'string'}, {'scalartext'});
if ~iscell(scenarios)
    scenarios = {scenarios};
end

n = numel(scenarios);
required = {'runs', 'metrics', 'failures'};
if ~isstruct(batch) || ~all(isfield(batch, required)) || ...
        numel(batch.runs) ~= n || numel(batch.metrics) ~= n || numel(batch.failures) ~= n
    error('agc:io:saveBatchSuite:BatchContract', ...
        'Batch runs, metrics, and failures must match the number of scenarios.');
end

suiteDirectory = char(suiteDirectory);
if ~isfolder(suiteDirectory), mkdir(suiteDirectory); end
saved = repmat(struct('name', '', 'directory', '', 'saved', false, 'failure', []), n, 1);

for k = 1:n
    controller = scenarios{k}.controller;
    name = sprintf('%s_%s', lower(char(string(controller.mode))), ...
        lower(char(string(controller.coriolis))));
    resultDirectory = fullfile(suiteDirectory, name);
    saved(k) = struct('name', name, 'directory', resultDirectory, ...
        'saved', false, 'failure', batch.failures{k});

    if isempty(batch.failures{k})
        agc.io.saveRun(resultDirectory, scenarios{k}, batch.runs{k}, batch.metrics{k});
        saved(k).saved = true;
    end
end
end
