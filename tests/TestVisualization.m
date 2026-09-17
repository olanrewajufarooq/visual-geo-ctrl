classdef TestVisualization < matlab.unittest.TestCase
    methods (Test)
        function paperFiguresExportAllStaticAssets(testCase)
            suiteDirectory = tempname;
            cleanup = onCleanup(@() removeDirectory(suiteDirectory)); %#ok<NASGU>
            variants = {'nominal', 'c1'; 'nominal', 'c2'; 'euclidean', 'c1'; ...
                'euclidean', 'c2'; 'bregman', 'c1'; 'bregman', 'c2'};

            for k = 1:size(variants, 1)
                resultDirectory = fullfile(suiteDirectory, sprintf('%s_%s', variants{k,:}));
                [scenario, run, metrics] = fixture(variants{k,1}, variants{k,2});
                agc.io.saveRun(resultDirectory, scenario, run, metrics);
            end

            output = agc.viz.paperFigures(suiteDirectory, struct('visible', false));

            expected = ["trajectory", "tracking", "transverse", ...
                "adaptive_comparison", "performance_summary", "adaptation"];
            testCase.verifyEqual(string({output.name}), expected);
            for name = expected
                testCase.verifyTrue(isfile(fullfile(output(1).directory, name + ".png")));
                testCase.verifyTrue(isfile(fullfile(output(1).directory, name + ".pdf")));
            end
        end
    end
end

function [scenario, run, metrics] = fixture(mode, coriolis)
t = (0:0.1:0.2).';
n = numel(t);
H = repmat(eye(4), 1, 1, n);
Hd = H;
for k = 1:n
    H(1:3,4,k) = [0.1 * k; 0.02 * k; 0.01 * k];
    Hd(1:3,4,k) = [0.1 * k; 0; 0];
end
pi = [2; 0; 0; -0.01; 0.1; 0.12; 0.14; 0.001; 0.002; -0.001];
estimatePi = repmat(pi.', n, 1);
if ~strcmpi(mode, 'nominal')
    estimatePi = estimatePi .* (1 + 0.02 * exp(-t));
end
run = struct('t', t, 'H', H, 'V', zeros(n,6), 'Hdesired', Hd, 'Vdesired', zeros(n,6), ...
    'wrench', repmat([0, 0, 0, 0, 0, 19.62], n, 1), 's', repmat([0, 0, 0, 0.1, 0, 0], n, 1), ...
    'Psi', exp(-t), 'Vs', 0.1 * exp(-t), 'estimatePi', estimatePi, ...
    'minPseudoEigenvalue', ternary(strcmpi(mode, 'bregman'), 0.01 * ones(n,1), nan(n,1)), ...
    'mode', mode, 'coriolis', coriolis, 'finalEstimate', pi);
scenario = struct('plantPi', pi);
metrics = agc.sim.metrics(run);
end

function value = ternary(condition, trueValue, falseValue)
if condition, value = trueValue; else, value = falseValue; end
end

function removeDirectory(directory)
if isfolder(directory), rmdir(directory, 's'); end
end
