function scenarios = applyGainBlock(blockCandidate, incumbentCandidate, block, scenarios)
%APPLYGAINBLOCK Decode one block while retaining all fixed incumbent gains.

if ~iscell(scenarios), scenarios = {scenarios}; end
if isempty(scenarios)
    error('agc:opt:applyGainBlock:EmptyScenarios', 'At least one scenario is required.');
end

mode = scenarios{1}.controller.mode;
indices = agc.opt.gainBlockIndices(mode, block);
incumbentCandidate = incumbentCandidate(:).';
blockCandidate = blockCandidate(:).';
if numel(incumbentCandidate) ~= numel(agc.opt.gainBlockIndices(mode, 'all'))
    error('agc:opt:applyGainBlock:IncumbentWidth', 'incumbentCandidate has the wrong width.');
end
if numel(blockCandidate) ~= numel(indices)
    error('agc:opt:applyGainBlock:BlockWidth', 'blockCandidate has the wrong width for %s.', block);
end

candidate = incumbentCandidate;
candidate(indices) = blockCandidate;
scenarios = agc.opt.applyScenarioGains(candidate, scenarios);
end
