function profile = profileBregmanGain(baseScenario, incumbentCandidate, seedValues, weights, useParallel)
%PROFILEBREGMANGAIN Evaluate the scalar gammaB block on a diagnostic grid.

grid = agc.opt.bregmanGammaGrid(seedValues);
records = cell(numel(grid), 1);
useParallel = logical(useParallel) && ~isempty(ver('parallel'));
if useParallel
    incumbentConstant = parallel.pool.Constant(incumbentCandidate(:).');
    parfor k = 1:numel(grid)
        candidate = incumbentConstant.Value;
        candidate(16) = grid(k);
        records{k} = agc.opt.evaluateScenarioCandidate(candidate, baseScenario, weights, false, 'adaptive');
    end
else
    for k = 1:numel(grid)
        candidate = incumbentCandidate(:).';
        candidate(16) = grid(k);
        records{k} = agc.opt.evaluateScenarioCandidate(candidate, baseScenario, weights, false, 'adaptive');
    end
end

cost = cellfun(@(record) record.cost, records);
failed = cellfun(@(record) record.failed, records);
valid = find(~failed & isfinite(cost));
best = struct('candidate', incumbentCandidate(:).', 'cost', Inf, 'failed', true, ...
    'label', 'adaptive', 'rawCandidate', incumbentCandidate(:).', 'gains', struct(), 'detail', struct());
if ~isempty(valid)
    [~, localIndex] = min(cost(valid));
    best = records{valid(localIndex)};
end
% Preserve the complete scalar cost profile but retain only the winning run
% diagnostic; keeping every 30 s replay log would make artifacts impractical.
profile = struct('logGammaB', grid(:), 'gammaB', 10 .^ grid(:), 'cost', cost(:), ...
    'failed', failed(:), 'best', best, 'parallel', useParallel);
end
