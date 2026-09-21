function weights = objectiveWeights()
%OBJECTIVEWEIGHTS Return the shared optimizer trade-off convention.

weights = struct('position', 2, 'attitude', 2, 'mass', 2.5, 'cog', 2.5, ...
    'linVel', 0.5, 'angVel', 0.5, 'inertia', 1.5, ...
    'effort', 0.01, 'failure', 1e6);
end
