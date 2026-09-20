function grid = bregmanGammaGrid(seedValues)
%BREGMANGAMMAGRID Build the log10 gammaB diagnostic profile grid.

if nargin < 1 || isempty(seedValues), seedValues = []; end
validateattributes(seedValues, {'numeric'}, {'real', 'finite', 'positive'});
[lowerBound, upperBound] = agc.opt.gainBounds('bregman');
grid = linspace(lowerBound(16), upperBound(16), 21);
seedCoordinates = log10(seedValues(:).');
seedCoordinates = min(max(seedCoordinates, lowerBound(16)), upperBound(16));
grid = unique([grid, seedCoordinates], 'sorted');
end
