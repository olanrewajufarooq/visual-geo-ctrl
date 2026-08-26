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
    end
end
