classdef TestBatchPersistence < matlab.unittest.TestCase
    methods (Test)
        function savesEverySuccessfulVariantInNamedDirectories(testCase)
            suiteDirectory = tempname;
            cleanup = onCleanup(@() removeDirectory(suiteDirectory)); %#ok<NASGU>
            scenarios = {fixture('nominal', 'c1'), fixture('bregman', 'c2')};
            batch = struct('runs', {{struct('id', 1), struct('id', 2)}}, ...
                'metrics', {{struct('cost', 3), struct('cost', 4)}}, ...
                'failures', {{[], struct('message', 'failed')}});

            saved = agc.io.saveBatchSuite(suiteDirectory, scenarios, batch);

            testCase.verifyEqual(string({saved.name}), ["nominal_c1", "bregman_c2"]);
            testCase.verifyTrue(saved(1).saved);
            testCase.verifyFalse(saved(2).saved);
            testCase.verifyTrue(isfile(fullfile(suiteDirectory, 'nominal_c1', 'run.mat')));
            payload = agc.io.loadRun(fullfile(suiteDirectory, 'nominal_c1'));
            testCase.verifyEqual(payload.run.id, 1);
        end

        function resolvesNewestCompleteSuiteBelowResultsRoot(testCase)
            rootDirectory = tempname;
            cleanup = onCleanup(@() removeDirectory(rootDirectory)); %#ok<NASGU>
            createSuite(rootDirectory, '20260917_100000', false);
            expected = createSuite(rootDirectory, '20260917_110000', true);

            actual = agc.io.resolveResultSuite(rootDirectory);

            testCase.verifyEqual(actual, expected);
        end
    end
end

function scenario = fixture(mode, coriolis)
scenario = struct('controller', struct('mode', mode, 'coriolis', coriolis));
end

function removeDirectory(directory)
if isfolder(directory), rmdir(directory, 's'); end
end

function directory = createSuite(rootDirectory, name, complete)
directory = fullfile(rootDirectory, name);
variants = {'nominal_c1', 'nominal_c2', 'euclidean_c1', 'euclidean_c2', 'bregman_c1', 'bregman_c2'};
if ~complete, variants = variants(1:5); end
for k = 1:numel(variants)
    resultDirectory = fullfile(directory, variants{k});
    mkdir(resultDirectory);
    payload = k; %#ok<NASGU>
    save(fullfile(resultDirectory, 'run.mat'), 'payload');
end
end
