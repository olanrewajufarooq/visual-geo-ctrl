function [time, values] = zohTrace(time, values)
%ZOHTRACE Reduce a zero-order-held signal to its transition samples.

validateattributes(time, {'numeric'}, {'real', 'finite', 'vector', 'nonempty'});
validateattributes(values, {'numeric'}, {'real', 'finite', '2d', 'nrows', numel(time)});

time = time(:);
if isscalar(time), return; end

changed = any(values(2:end,:) ~= values(1:end-1,:), 2);
index = unique([1; find(changed) + 1; numel(time)], 'stable');
time = time(index);
values = values(index,:);
end
