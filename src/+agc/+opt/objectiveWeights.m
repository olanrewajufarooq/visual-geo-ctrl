function weights = objectiveWeights()
%OBJECTIVEWEIGHTS Return the shared optimizer trade-off convention.

weights = struct('position', 1, 'attitude', 1, 'linVel', 1, 'angVel', 1, ...
    'effort', 0.01, 'estimation', 0.1, 'failure', 1e6);
end
