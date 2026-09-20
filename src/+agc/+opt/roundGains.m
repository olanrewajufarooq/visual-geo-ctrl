function gains = roundGains(gains, significantFigures)
%ROUNDGAINS Round every published gain to a fixed number of significant figures.
%
% Significant figures keep small adaptation gains readable without rounding
% them to zero, while limiting the precision written to optimization artifacts
% and the optimized-gain registry.

if nargin < 2 || isempty(significantFigures), significantFigures = 4; end
validateattributes(significantFigures, {'numeric'}, {'scalar', 'integer', 'positive'});
fields = {'KRdiag', 'Kxidiag', 'LambdaDiag', 'kd', 'ks', 'alpha', 'gammaE', 'gammaB'};
for k = 1:numel(fields)
    field = fields{k};
    if ~isfield(gains, field)
        error('agc:opt:roundGains:Field', 'gains is missing %s.', field);
    end
    gains.(field) = roundSignificant(gains.(field), significantFigures);
end
end

function value = roundSignificant(value, significantFigures)
value = double(value);
nonzero = value ~= 0;
scale = ones(size(value));
scale(nonzero) = 10 .^ (significantFigures - 1 - floor(log10(abs(value(nonzero)))));
value(nonzero) = round(value(nonzero) .* scale(nonzero)) ./ scale(nonzero);
end
