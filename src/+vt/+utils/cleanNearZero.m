function out = cleanNearZero(in, tol)
%CLEANNEARZERO Replace values near zero with exact zero
    if nargin < 2
        tol = 1e-12;
    end

    if isstruct(in)
        out = in;
        fields = fieldnames(in);
        for i = 1:numel(fields)
            out.(fields{i}) = vt.utils.cleanNearZero(in.(fields{i}), tol);
        end
    elseif isnumeric(in)
        out = in;
        nearZero = abs(in) < tol;
        out(nearZero) = 0;
    else
        out = in;
    end
end
