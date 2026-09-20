function [lowerBound, upperBound] = gainBounds(mode)
%GAINBOUNDS Default all-gain particle-swarm bounds in optimizer coordinates.
%
% Positive gains use log10 coordinates. Alpha uses physical coordinates.

mode = lower(char(string(mode)));
if ~ismember(mode, {'nominal', 'euclidean', 'bregman'})
    error('agc:opt:gainBounds:UnknownMode', 'Unknown controller mode: %s.', mode);
end

% Gain-family bounds keep random particles in a numerically meaningful
% regime while preserving the paper constraints kd > 1/2 and 0 < alpha < 1.
positiveLower = [log10(1e-3) * ones(1,3), log10(1e-2) * ones(1,3), ...
    log10(1e-2) * ones(1,6), log10(0.55), log10(1e-3)];
positiveUpper = [log10(30) * ones(1,3), log10(300) * ones(1,3), ...
    log10(100) * ones(1,3), log10(10) * ones(1,3), log10(100), log10(50)];
lowerBound = [positiveLower, 0.05];
upperBound = [positiveUpper, 0.95];
if strcmp(mode, 'euclidean')
    lowerBound = [lowerBound, repmat(log10(1e-5), 1, 10)];
    upperBound = [upperBound, zeros(1, 10)];
elseif strcmp(mode, 'bregman')
    lowerBound(end + 1) = log10(1e-5);
    upperBound(end + 1) = log10(1e-1);
end
end
