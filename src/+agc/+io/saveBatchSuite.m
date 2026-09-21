function saved = saveBatchSuite(suiteDirectory, scenarios, batch)
%SAVEBATCHSUITE Persist completed runs and finite runtime-failure prefixes.
%
% A suite is directly consumable by agc.viz.paperFigures: every available run
% is stored as <suite>/<mode>_<coriolis>/run.mat. A failed finite prefix keeps
% failure metadata but deliberately has no completed-run metric.

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
saved = repmat(struct('name', '', 'directory', '', 'saved', false, ...
    'successful', false, 'failure', []), n, 1);

for k = 1:n
    controller = scenarios{k}.controller;
    name = sprintf('%s_%s', lower(char(string(controller.mode))), ...
        lower(char(string(controller.coriolis))));
    resultDirectory = fullfile(suiteDirectory, name);
    saved(k) = struct('name', name, 'directory', resultDirectory, ...
        'saved', false, 'successful', isempty(batch.failures{k}), ...
        'failure', batch.failures{k});

    if ~isempty(batch.runs{k})
        agc.io.saveRun(resultDirectory, scenarios{k}, batch.runs{k}, ...
            batch.metrics{k}, batch.failures{k});
        saved(k).saved = true;
    end
end
end
