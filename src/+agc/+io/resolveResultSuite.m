function suiteDirectory = resolveResultSuite(path)
%RESOLVERESULTSUITE Return a saved suite or the newest child suite.
%
% Passing results/<timestamp> is explicit and returns that directory. Passing
% results/ selects the lexically newest direct child containing at least one
% saved run. This permits post-processing successful members of a partial
% batch after other variants fail.

validateattributes(path, {'char', 'string'}, {'nonempty', 'scalartext'});
path = char(path);
if ~isfolder(path)
    error('agc:io:resolveResultSuite:NotFound', 'Result directory does not exist: %s', path);
end
if isResultSuite(path)
    suiteDirectory = path;
    return;
end

children = dir(path);
names = string({children([children.isdir]).name});
names = sort(names(~startsWith(names, '.')), 'descend');
for name = names
    candidate = fullfile(path, name);
    if isResultSuite(candidate)
        suiteDirectory = char(candidate);
        return;
    end
end
error('agc:io:resolveResultSuite:NoSavedSuite', ...
    'No result suite containing a saved run was found below: %s', path);
end

function value = isResultSuite(directory)
children = dir(directory);
value = any(arrayfun(@(child) child.isdir && ~startsWith(child.name, '.') && ...
    isfile(fullfile(directory, child.name, 'run.mat')), children));
end
