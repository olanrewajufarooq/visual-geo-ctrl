function values = logTrace(values)
%LOGTRACE Preserve positive logarithmic samples and show zeros as gaps.

validateattributes(values, {'numeric'}, {'real'});
values(values <= 0) = NaN;
end
