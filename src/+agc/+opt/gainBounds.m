function [lowerBound, upperBound] = gainBounds(mode)
%GAINBOUNDS Default all-gain particle-swarm bounds in optimizer coordinates.
%
% Positive gains use log10 coordinates. Alpha uses physical coordinates.

mode = lower(char(string(mode)));
if ~ismember(mode, {'nominal', 'euclidean', 'bregman'})
    error('agc:opt:gainBounds:UnknownMode', 'Unknown controller mode: %s.', mode);
end

positiveLower = [repmat(-2, 1, 12), -3, -3];
positiveUpper = [repmat(2, 1, 12), 2, 2];
lowerBound = [positiveLower, 0.1];
upperBound = [positiveUpper, 0.9];
if strcmp(mode, 'euclidean')
    lowerBound = [lowerBound, repmat(-8, 1, 10)];
    upperBound = [upperBound, repmat(1, 1, 10)];
elseif strcmp(mode, 'bregman')
    lowerBound(end + 1) = -8;
    upperBound(end + 1) = 1;
end
end
