function result = optimizeGainBlock(baseScenario, incumbentCandidate, block, seedCandidates, options)
%OPTIMIZEGAINBLOCK Run particle swarm over one gain block only.

mode = baseScenario.controller.mode;
indices = agc.opt.gainBlockIndices(mode, block);
[lowerBound, upperBound] = agc.opt.gainBounds(mode);
incumbentCandidate = incumbentCandidate(:).';
seedCandidates = seedCandidates(:, :);
if size(seedCandidates, 2) ~= numel(incumbentCandidate)
    error('agc:opt:optimizeGainBlock:SeedWidth', 'Every seed must be a complete optimizer candidate.');
end

blockSeeds = unique(seedCandidates(:, indices), 'rows', 'stable');
options.initialPoints = blockSeeds;
decode = @(candidate, scenarios) agc.opt.applyGainBlock(candidate, incumbentCandidate, block, scenarios);
partial = agc.opt.optimize({baseScenario}, decode, lowerBound(indices), upperBound(indices), options);

candidate = incumbentCandidate;
candidate(indices) = partial.candidate;
result = partial;
result.block = block;
result.blockIndices = indices;
result.blockCandidate = partial.candidate;
result.candidate = candidate;
end
