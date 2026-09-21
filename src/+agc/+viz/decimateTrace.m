function [time, values] = decimateTrace(time, values, maxSamples)
%DECIMATETRACE Bound a continuous displayed trace while retaining endpoints.

if nargin < 3 || isempty(maxSamples), maxSamples = 2000; end
validateattributes(time, {'numeric'}, {'real', 'finite', 'vector', 'nonempty'});
validateattributes(values, {'numeric'}, {'real', '2d', 'nrows', numel(time)});
validateattributes(maxSamples, {'numeric'}, {'real', 'finite', 'integer', 'positive', 'scalar'});

time = time(:);
if numel(time) <= maxSamples, return; end

index = unique(round(linspace(1, numel(time), maxSamples)), 'stable').';
time = time(index);
values = values(index,:);
end
