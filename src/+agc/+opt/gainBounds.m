function [lowerBound, upperBound] = gainBounds(mode)
%GAINBOUNDS Default all-gain particle-swarm bounds in optimizer coordinates.
%
% Positive gains use log10 coordinates. Alpha uses physical coordinates.

mode = lower(char(string(mode)));
if ~ismember(mode, {'nominal', 'euclidean', 'bregman'})
    error('agc:opt:gainBounds:UnknownMode', 'Unknown controller mode: %s.', mode);
end

% Controller gains use a wide shared range [1e-4, 1e4]. This avoids
% artificial optima at the old 1e-2/1e2 limits while retaining positivity.
positiveLower = [-4 * ones(1, 12), log10(0.5001), -4];
positiveUpper = 4 * ones(1, 14);
lowerBound = [positiveLower, 0.01];
upperBound = [positiveUpper, 0.99];
if strcmp(mode, 'euclidean')
    lowerBound = [lowerBound, repmat(-8, 1, 10)];
    upperBound = [upperBound, repmat(3, 1, 10)];
elseif strcmp(mode, 'bregman')
    lowerBound(end + 1) = -8;
    upperBound(end + 1) = 3;
end
end
