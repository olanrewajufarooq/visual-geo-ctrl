function options = optimizationOptions(options)
%OPTIMIZATIONOPTIONS Normalize all particle-swarm search and stop settings.
%
% The default stop rule declares convergence after ten consecutive stalled
% iterations whose best objective change is within 1e-3. A full search still
% cannot exceed twenty iterations unless a caller explicitly changes it.

if nargin < 1 || isempty(options), options = struct(); end
if ~isstruct(options)
    error('agc:opt:optimizationOptions:Type', 'options must be a struct.');
end
defaults = struct('weights', agc.opt.objectiveWeights(), 'scales', agc.opt.objectiveScales(), ...
    'swarmSize', 30, 'maxIterations', 20, ...
    'parallel', true, 'functionTolerance', 1e-3, 'maxStallIterations', 10, ...
    'initialPoints', []);
fields = fieldnames(defaults);
for k = 1:numel(fields)
    field = fields{k};
    if ~isfield(options, field), options.(field) = defaults.(field); end
end

validateattributes(options.swarmSize, {'numeric'}, {'scalar', 'integer', 'positive'});
validateattributes(options.maxIterations, {'numeric'}, {'scalar', 'integer', 'positive'});
validateattributes(options.parallel, {'logical', 'numeric'}, {'scalar'});
validateattributes(options.functionTolerance, {'numeric'}, {'real', 'finite', 'positive', 'scalar'});
validateattributes(options.maxStallIterations, {'numeric'}, {'scalar', 'integer', 'positive'});
validateattributes(options.initialPoints, {'numeric'}, {'real', 'finite'});
end
