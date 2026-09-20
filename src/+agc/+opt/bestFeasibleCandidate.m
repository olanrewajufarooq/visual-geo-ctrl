function incumbent = bestFeasibleCandidate(incumbent, contender)
%BESTFEASIBLECANDIDATE Retain the lower-cost feasible optimization result.

required = {'candidate', 'cost', 'failed', 'label'};
if ~isstruct(incumbent) || ~all(isfield(incumbent, required)) || ...
        ~isstruct(contender) || ~all(isfield(contender, required))
    error('agc:opt:bestFeasibleCandidate:Contract', 'Both inputs must be candidate records.');
end
if ~contender.failed && isfinite(contender.cost) && ...
        (incumbent.failed || ~isfinite(incumbent.cost) || contender.cost < incumbent.cost)
    incumbent = contender;
end
end
