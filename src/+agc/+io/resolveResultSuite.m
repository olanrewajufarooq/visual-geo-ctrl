function suiteDirectory = resolveResultSuite(path)
%RESOLVERESULTSUITE Return a complete saved suite or the newest child suite.
%
% Passing results/<timestamp> is explicit and returns that directory. Passing
% results/ selects the lexically newest direct child containing every paper
% controller variant, which makes post-processing the latest batch convenient.

validateattributes(path, {'char', 'string'}, {'nonempty', 'scalartext'});
path = char(path);
if ~isfolder(path)
    error('agc:io:resolveResultSuite:NotFound', 'Result directory does not exist: %s', path);
end
if isCompleteSuite(path)
    suiteDirectory = path;
    return;
end

children = dir(path);
names = string({children([children.isdir]).name});
names = sort(names(~startsWith(names, '.')), 'descend');
for name = names
    candidate = fullfile(path, name);
    if isCompleteSuite(candidate)
        suiteDirectory = char(candidate);
        return;
    end
end
error('agc:io:resolveResultSuite:NoCompleteSuite', ...
    'No complete six-variant result suite was found below: %s', path);
end

function value = isCompleteSuite(directory)
variants = ["nominal_c1", "nominal_c2", "euclidean_c1", "euclidean_c2", ...
    "bregman_c1", "bregman_c2"];
value = all(arrayfun(@(name) isfile(fullfile(directory, name, 'run.mat')), variants));
end
