classdef TestGainRegistry < matlab.unittest.TestCase
    methods (TestMethodSetup)
        function addConfigurationPath(~)
            addpath(fullfile(agc.io.repositoryRoot(), 'config'));
        end
    end

    methods (Test)
        function manualAndOptimizedRegistriesCoverEveryScenario(testCase)
            modes = ["nominal", "euclidean", "bregman"];
            forms = ["c1", "c2"];
            required = {'KRdiag', 'Kxidiag', 'LambdaDiag', 'kd', 'ks', 'alpha', 'gammaE', 'gammaB'};

            for mode = modes
                for form = forms
                    manual = manual_gains(mode, form);
                    optimized = optimized_gains(mode, form);

                    testCase.verifyTrue(all(isfield(manual, required)));
                    testCase.verifyTrue(all(isfield(optimized, required)));
                    testCase.verifySize(manual.KRdiag, [1, 3]);
                    testCase.verifySize(manual.Kxidiag, [1, 3]);
                    testCase.verifySize(manual.LambdaDiag, [1, 6]);
                    testCase.verifySize(manual.gammaE, [10, 1]);
                    testCase.verifySize(optimized.KRdiag, [1, 3]);
                    testCase.verifySize(optimized.Kxidiag, [1, 3]);
                    testCase.verifySize(optimized.LambdaDiag, [1, 6]);
                    testCase.verifySize(optimized.gammaE, [10, 1]);
                    testCase.verifyGreaterThan(min([manual.KRdiag, manual.Kxidiag, manual.LambdaDiag, ...
                        manual.kd, manual.ks, manual.gammaB, manual.gammaE.']), 0);
                    testCase.verifyGreaterThan(min([optimized.KRdiag, optimized.Kxidiag, optimized.LambdaDiag, ...
                        optimized.kd, optimized.ks, optimized.gammaB, optimized.gammaE.']), 0);
                    testCase.verifyGreaterThan(manual.alpha, 0);
                    testCase.verifyLessThan(manual.alpha, 1);
                    testCase.verifyGreaterThan(optimized.alpha, 0);
                    testCase.verifyLessThan(optimized.alpha, 1);
                end
            end
        end

        function allGainDecoderAppliesAdaptiveLogSpaceCandidate(testCase)
            scenario = agc.sim.defaultScenario('lemniscate_01_auto', 'euclidean', 'c1', 0.1, 'manual');
            positive = [1.1, 1.2, 1.3, 2.1, 2.2, 2.3, 3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 4.1, 4.2];
            gammaE = (1:10).' * 1e-3;
            candidate = [log10(positive), 0.4, log10(gammaE.')];

            decoded = agc.opt.applyScenarioGains(candidate, {scenario});
            controller = decoded{1}.controller;

            testCase.verifyEqual(controller.KR, diag(positive(1:3)), 'AbsTol', 1e-12);
            testCase.verifyEqual(controller.Kxi, diag(positive(4:6)), 'AbsTol', 1e-12);
            testCase.verifyEqual(controller.Lambda, diag(positive(7:12)), 'AbsTol', 1e-12);
            testCase.verifyEqual([controller.kd, controller.ks, controller.alpha], ...
                [positive(13), positive(14), 0.4], 'AbsTol', 1e-12);
            testCase.verifyEqual(controller.gammaE, gammaE, 'AbsTol', 1e-12);
        end

        function allGainBoundsMatchEachModeParameterization(testCase)
            [nominalLower, nominalUpper] = agc.opt.gainBounds('nominal');
            [euclideanLower, euclideanUpper] = agc.opt.gainBounds('euclidean');
            [bregmanLower, bregmanUpper] = agc.opt.gainBounds('bregman');

            testCase.verifySize(nominalLower, [1, 15]);
            testCase.verifySize(nominalUpper, [1, 15]);
            testCase.verifySize(euclideanLower, [1, 25]);
            testCase.verifySize(euclideanUpper, [1, 25]);
            testCase.verifySize(bregmanLower, [1, 16]);
            testCase.verifySize(bregmanUpper, [1, 16]);
            testCase.verifyEqual(nominalLower(15), 0.1);
            testCase.verifyEqual(nominalUpper(15), 0.9);
        end

        function promotionRewritesOnlySelectedOptimizedEntry(testCase)
            directory = tempname;
            mkdir(directory);
            cleanup = onCleanup(@() removeDirectory(directory)); %#ok<NASGU>
            targetFile = fullfile(directory, 'optimized_gains.m');
            custom = manual_gains('euclidean', 'c1');
            custom.kd = 7.5;

            agc.io.promoteOptimizedGains('euclidean', 'c1', custom, struct('cost', 12.3), targetFile);

            clear optimized_gains
            addpath(directory, '-begin');
            promoted = optimized_gains('euclidean', 'c1');
            untouched = optimized_gains('bregman', 'c2');
            rmpath(directory);
            clear optimized_gains

            testCase.verifyEqual(promoted.kd, custom.kd, 'AbsTol', 1e-12);
            testCase.verifyEqual(untouched, manual_gains('bregman', 'c2'), 'AbsTol', 1e-12);
        end
    end
end

function removeDirectory(directory)
if isfolder(directory), rmdir(directory, 's'); end
end
