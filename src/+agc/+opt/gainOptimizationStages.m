function stages = gainOptimizationStages(mode)
%GAINOPTIMIZATIONSTAGES Return the one-pass staged optimization schedule.

mode = lower(char(string(mode)));
switch mode
    case 'nominal'
        stages = {'all'};
    case {'euclidean', 'bregman'}
        stages = {'all', 'nonadaptive', 'adaptive', 'all'};
    otherwise
        error('agc:opt:gainOptimizationStages:Mode', 'Unknown controller mode: %s.', mode);
end
end
