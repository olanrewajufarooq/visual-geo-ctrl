function promoteOptimizedGains(mode, coriolis, gains, metadata, targetFile)
%PROMOTEOPTIMIZEDGAINS Persist one scenario's best gains in the registry.
%
% The generated function is deterministic and self-contained so it remains
% safe to load on MATLAB parallel workers without external state.

if nargin < 4 || isempty(metadata), metadata = struct(); end
if nargin < 5 || isempty(targetFile)
    targetFile = fullfile(agc.io.repositoryRoot(), 'config', 'optimized_gains.m');
end
validateattributes(targetFile, {'char', 'string'}, {'scalartext'});
if ~isstruct(metadata)
    error('agc:io:promoteOptimizedGains:Metadata', 'metadata must be a struct.');
end
validateGains(gains);

modes = ["nominal", "euclidean", "bregman"];
forms = ["c1", "c2"];
entries = struct('mode', {}, 'coriolis', {}, 'gains', {});
for currentMode = modes
    for currentForm = forms
        entries(end + 1) = struct('mode', currentMode, 'coriolis', currentForm, ... %#ok<AGROW>
            'gains', optimized_gains(currentMode, currentForm));
    end
end

index = find([entries.mode] == string(mode) & [entries.coriolis] == string(coriolis));
if numel(index) ~= 1
    error('agc:io:promoteOptimizedGains:UnknownScenario', 'Unknown scenario key.');
end
entries(index).gains = gains;

targetFile = char(targetFile);
targetDirectory = fileparts(targetFile);
if ~isfolder(targetDirectory), mkdir(targetDirectory); end
writeRegistry(targetFile, entries, metadata);
end

function validateGains(gains)
required = {'KRdiag', 'Kxidiag', 'LambdaDiag', 'kd', 'ks', 'alpha', 'gammaE', 'gammaB'};
if ~isstruct(gains) || ~all(isfield(gains, required))
    error('agc:io:promoteOptimizedGains:GainFields', 'gains has missing required fields.');
end
validateattributes(gains.KRdiag, {'numeric'}, {'real', 'finite', 'size', [1 3], 'positive'});
validateattributes(gains.Kxidiag, {'numeric'}, {'real', 'finite', 'size', [1 3], 'positive'});
validateattributes(gains.LambdaDiag, {'numeric'}, {'real', 'finite', 'size', [1 6], 'positive'});
validateattributes(gains.kd, {'numeric'}, {'real', 'finite', 'positive', 'scalar'});
validateattributes(gains.ks, {'numeric'}, {'real', 'finite', 'positive', 'scalar'});
validateattributes(gains.alpha, {'numeric'}, {'real', 'finite', '>', 0, '<', 1, 'scalar'});
validateattributes(gains.gammaE, {'numeric'}, {'real', 'finite', 'positive', 'size', [10, 1]});
validateattributes(gains.gammaB, {'numeric'}, {'real', 'finite', 'positive', 'scalar'});
end

function writeRegistry(targetFile, entries, metadata)
timestamp = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));
lines = { ...
    'function gains = optimized_gains(mode, coriolis)', ...
    '%OPTIMIZED_GAINS Automatically maintained per-scenario gain registry.', ...
    ['% Last promotion: ', timestamp, '.'], ...
    'key = sprintf(''%s_%s'', lower(char(string(mode))), lower(char(string(coriolis))));', ...
    'switch key'};

for k = 1:numel(entries)
    key = sprintf('%s_%s', entries(k).mode, entries(k).coriolis);
    lines{end + 1} = ['    case ''', key, '''']; %#ok<AGROW>
    lines{end + 1} = ['        gains = ', renderGains(entries(k).gains), ';']; %#ok<AGROW>
end

lines = [lines, { ...
    '    otherwise', ...
    '        error(''agc:config:optimized_gains:UnknownScenario'', ''Unknown scenario key: %s.'', key);', ...
    'end', ...
    'end'}];

% Metadata belongs to the timestamped optimization artifact; accepting it
% here makes the promotion API self-documenting without bloating the registry.
if ~isempty(fieldnames(metadata)) %#ok<NASGU>
end

file = fopen(targetFile, 'w');
if file < 0
    error('agc:io:promoteOptimizedGains:OpenFailed', 'Cannot write %s.', targetFile);
end
cleanup = onCleanup(@() fclose(file)); %#ok<NASGU>
fprintf(file, '%s\n', strjoin(lines, newline));
end

function text = renderGains(gains)
text = sprintf(['struct(''KRdiag'', %s, ''Kxidiag'', %s, ''LambdaDiag'', %s, ', ...
    '''kd'', %.17g, ''ks'', %.17g, ''alpha'', %.17g, ''gammaE'', %s, ''gammaB'', %.17g)'], ...
    mat2str(gains.KRdiag, 17), mat2str(gains.Kxidiag, 17), mat2str(gains.LambdaDiag, 17), ...
    gains.kd, gains.ks, gains.alpha, mat2str(gains.gammaE, 17), gains.gammaB);
end
