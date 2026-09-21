function scales = objectiveScales()
%OBJECTIVESCALES Return acceptable-error scales for dimensionless costs.

scales = struct('position', 0.05, 'attitude', 0.05, ...
    'mass', 0.05, 'cog', 0.01, 'linVel', 0.20, 'angVel', 0.50, ...
    'inertia', 0.05, 'effort', 50);
end
