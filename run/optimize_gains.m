% OPTIMIZE_GAINS Run staged derivative-free block-coordinate gain optimization.
%
% The schedule is joint all-gain search, non-adaptive search, adaptive search,
% then a final all-gain polish. Each stage retains the best feasible incumbent.

%% User settings

replayId = 'lemniscate_01_auto';

% Use '' for every option, one char/string for one option, or a cell array.
% Examples: mode = ''; coriolis = ''; or mode = {'bregman','euclidean'};
mode = '';

coriolis = '';

duration = 30;

useParallel = true;

swarmSize = 50;

maxIterations = 100;

functionTolerance = 1e-3;

maxStallIterations = 25;

promoteBest = true;

%% Expand requested scenarios and set the common objective

startup;

variants = agc.opt.expandScenarioSelection(mode, coriolis);

weights = struct('position', 1, 'attitude', 1, 'effort', 1e-3, ...
    'estimation', 0.1, 'failure', 1e6);

root = agc.io.repositoryRoot();

stamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));

options = struct('weights', weights, 'swarmSize', swarmSize, ...
    'maxIterations', maxIterations, 'functionTolerance', functionTolerance, ...
    'maxStallIterations', maxStallIterations, 'parallel', useParallel);

%% Optimize each selected mode/Coriolis scenario without nested pools

% Each particle swarm parallelizes candidate evaluations internally. Keeping
% this outer loop serial avoids nested Parallel Computing Toolbox pools.
for variantIndex = 1:size(variants, 1)
    selectedMode = variants{variantIndex, 1};

    selectedCoriolis = variants{variantIndex, 2};

    fprintf('Optimizing %s/%s (%d of %d)\n', selectedMode, selectedCoriolis, ...
        variantIndex, size(variants, 1));

    % Evaluate manual and registered seed candidates.

    manualScenario = agc.sim.defaultScenario(replayId, selectedMode, selectedCoriolis, duration, 'manual');

    registeredScenario = agc.sim.defaultScenario(replayId, selectedMode, selectedCoriolis, duration, 'optimized');

    manual = agc.opt.evaluateScenarioCandidate(agc.opt.encodeScenarioGains(manualScenario), ...
        manualScenario, weights, false, 'manual');

    registered = agc.opt.evaluateScenarioCandidate(agc.opt.encodeScenarioGains(registeredScenario), ...
        manualScenario, weights, false, 'registered');

    incumbent = agc.opt.bestFeasibleCandidate(manual, registered);

    promoted = registered;

    resultDirectory = fullfile(root, 'results', 'optimization', stamp, ...
        sprintf('%s_%s', selectedMode, selectedCoriolis));

    if ~isfolder(resultDirectory)
        mkdir(resultDirectory);
    end

    stages = cell(0, 1);

    stageNames = agc.opt.gainOptimizationStages(selectedMode);

    % Stage seeds always include the manual, registered, and current best
    % complete candidates. Particleswarm randomizes the remaining population.
    for stageIndex = 1:numel(stageNames)
        stageName = stageNames{stageIndex};

        seedCandidates = unique([manual.candidate; registered.candidate; incumbent.candidate], 'rows', 'stable');

        fprintf('  Stage %d/%d: %s\n', stageIndex, numel(stageNames), stageName);

        if strcmp(stageName, 'adaptive') && strcmpi(selectedMode, 'bregman')
            profile = agc.opt.profileBregmanGain(manualScenario, incumbent.candidate, ...
                10 .^ seedCandidates(:,16), weights, useParallel);
            stageResult = struct('method', 'log-grid', 'profile', profile);
            contender = profile.best;
        else
            profile = [];
            stageResult = agc.opt.optimizeGainBlock(manualScenario, incumbent.candidate, ...
                stageName, seedCandidates, options);
            contender = agc.opt.evaluateScenarioCandidate(stageResult.candidate, ...
                manualScenario, weights, false, stageName);
        end

        incumbent = agc.opt.bestFeasibleCandidate(incumbent, contender);

        stages{end + 1, 1} = struct('name', stageName, 'result', stageResult, ...
            'contender', contender, 'incumbent', incumbent, 'profile', profile); %#ok<SAGROW>
        optimization = struct('mode', selectedMode, 'coriolis', selectedCoriolis, ...
            'weights', weights, 'options', options, 'manual', manual, 'registered', registered, ...
            'stages', {stages}, 'incumbent', incumbent);
        save(fullfile(resultDirectory, 'optimization.mat'), 'optimization', '-v7.3');

        if promoteBest && improved(incumbent, promoted)
            agc.io.promoteOptimizedGains(selectedMode, selectedCoriolis, incumbent.gains, ...
                struct('cost', incumbent.cost, 'artifact', resultDirectory, 'stage', stageName));

            clear optimized_gains

            promoted = incumbent;
        end

        fprintf('    incumbent cost: %.6g (%s)\n', incumbent.cost, incumbent.label);
    end

    fprintf('Best cost for %s/%s: %.6g\n', selectedMode, selectedCoriolis, incumbent.cost);
    fprintf('Saved staged optimization artifact: %s\n', resultDirectory);
end

function tf = improved(candidate, reference)
%IMPROVED Return true only for a strictly better feasible stage incumbent.

tf = ~candidate.failed && isfinite(candidate.cost) && ...
    (reference.failed || ~isfinite(reference.cost) || candidate.cost < reference.cost);
end
