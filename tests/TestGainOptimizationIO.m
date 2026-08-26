classdef TestGainOptimizationIO < matlab.unittest.TestCase
    methods (Test)
        function testGainTableUsesNamedPhysicalColumns(testCase)
            x = [1:18, -3];
            tableData = fth.opt.GainOptimizationIO.gainTable(7, 2.5, 12.0, x, 'improvement');

            testCase.verifyEqual(tableData.iteration, 7);
            testCase.verifyEqual(tableData.event, "improvement");
            testCase.verifyEqual(tableData.Kp_1, 1);
            testCase.verifyEqual(tableData.Kd_6, 12);
            testCase.verifyEqual(tableData.Gamma_1, 1e-3, 'AbsTol', 1e-12);
        end

        function testAppendImprovementIgnoresNonImprovement(testCase)
            history = fth.opt.GainOptimizationIO.emptyGainHistory(19);
            x = [1:18, -3];
            history = fth.opt.GainOptimizationIO.appendImprovement(history, 1, 3, 1, x);
            history = fth.opt.GainOptimizationIO.appendImprovement(history, 2, 3, 2, x);
            history = fth.opt.GainOptimizationIO.appendImprovement(history, 3, 2, 3, x);

            testCase.verifySize(history, [2, 23]);
            testCase.verifyEqual(history.iteration, [1; 3]);
            testCase.verifyEqual(history.best_cost, [3; 2]);
        end

        function testConvergenceWriterHasIterationHeader(testCase)
            folder = tempname;
            mkdir(folder);
            cleanup = onCleanup(@() rmdir(folder, 's')); %#ok<NASGU>
            fth.opt.GainOptimizationIO.writeConvergence(folder, [1, 4, 0.2; 2, 3, 0.4]);
            contents = fileread(fullfile(folder, 'convergence.csv'));
            testCase.verifySubstring(contents, 'iteration,best_cost,elapsed_seconds');
        end

        function testLoadBestVectorValidatesCompletedScenario(testCase)
            folder = tempname;
            mkdir(fullfile(folder, 'adaptive-basic-bregman-no-payload'));
            cleanup = onCleanup(@() rmdir(folder, 's')); %#ok<NASGU>
            scenario = struct('adaptation', 'bregman');
            bestX = [ones(1, 18), -3];
            completed = true;
            save(fullfile(folder, 'adaptive-basic-bregman-no-payload', 'optimizer_state.mat'), ...
                'bestX', 'completed', 'scenario');

            loaded = fth.opt.GainOptimizationIO.loadBestVector(folder, ...
                'adaptive-basic-bregman-no-payload', 19, 'bregman');
            testCase.verifyEqual(loaded, bestX);
        end

        function testLoadBestVectorRejectsIncompleteState(testCase)
            folder = tempname;
            mkdir(fullfile(folder, 'adaptive-basic-bregman-no-payload'));
            cleanup = onCleanup(@() rmdir(folder, 's')); %#ok<NASGU>
            bestX = [ones(1, 18), -3];
            completed = false;
            save(fullfile(folder, 'adaptive-basic-bregman-no-payload', 'optimizer_state.mat'), ...
                'bestX', 'completed');

            testCase.verifyError(@() fth.opt.GainOptimizationIO.loadBestVector(folder, ...
                'adaptive-basic-bregman-no-payload', 19, 'bregman'), ...
                'fth:GainOptimizationIO:IncompleteSource');
        end

        function testGainSourceConvertsDirectPhysicalGains(testCase)
            gains = struct('Kp', (1:6).', 'Kd', (7:12).', ...
                'lambda', (13:18).', 'Gamma', 1e-3);
            x = fth.opt.GainOptimizationSource.gainsToVector(gains, 'bregman');

            testCase.verifyEqual(x(1:18), 1:18, 'AbsTol', 1e-12);
            testCase.verifyEqual(x(19), -3, 'AbsTol', 1e-12);
        end

        function testGainSourceRejectsMissingLatestReport(testCase)
            root = tempname;
            mkdir(root);
            cleanup = onCleanup(@() rmdir(root, 's')); %#ok<NASGU>

            testCase.verifyError(@() fth.opt.GainOptimizationSource.loadVector( ...
                '', root, 'adaptive-basic-bregman-no-payload', 19, 'bregman'), ...
                'fth:GainOptimizationSource:MissingSource');
        end

        function testUpdateTopCandidatesKeepsThreeDistinctLowestCosts(testCase)
            existingX = [1, 1; 2, 2];
            existingCost = [3; 1];
            candidatesX = [3, 3; 1 + 1e-10, 1; 4, 4; 5, 5];
            candidatesCost = [2; 0.5; NaN; 4];

            [topX, topCost] = fth.opt.GainOptimizationIO.updateTopCandidates( ...
                existingX, existingCost, candidatesX, candidatesCost, [0, 0], [10, 10]);

            testCase.verifyEqual(topCost, [0.5; 1; 2]);
            testCase.verifyEqual(topX, [1, 1; 2, 2; 3, 3], 'AbsTol', 1e-9);
        end

        function testWriteTopGainsUsesRankedRows(testCase)
            folder = tempname;
            mkdir(folder);
            cleanup = onCleanup(@() rmdir(folder, 's')); %#ok<NASGU>
            scenario = struct('id', 'demo', 'adaptation', 'bregman', ...
                'coriolisForm', 'consistent', 'withPayload', false, 'paramInit', 'nominal');
            topX = [ones(1, 19); 2 * ones(1, 19); 3 * ones(1, 19)];
            topCost = [1; 2; 3];

            fth.opt.GainOptimizationIO.writeTopGains(folder, scenario, topX, topCost);
            contents = fileread(fullfile(folder, 'best_gains.csv'));

            testCase.verifySubstring(contents, 'rank_1');
            testCase.verifySubstring(contents, 'rank_3');
            testCase.verifySubstring(contents, 'Gamma_1');
            testCase.verifyTrue(isfile(fullfile(folder, 'best_gains.txt')));
            testCase.verifyTrue(isfile(fullfile(folder, 'best_gains.m')));
        end
    end
end
