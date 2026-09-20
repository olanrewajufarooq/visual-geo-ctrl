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

            names = string({output.name});

            % Comparison exports share axes only for like-for-like variants.
            comparisons = ["nominal_trajectory_c1_c2", ...
                "nominal_position_error_c1_c2", ...
                "euclidean_position_error_c1_c2", ...
                "bregman_position_error_c1_c2", ...
                "performance_position_rmse", ...
                "euclidean_vs_bregman_position_error_c1", ...
                "euclidean_vs_bregman_pseudo_inertia_margin_c2"];
            testCase.verifyTrue(all(ismember(comparisons, names)));

            % Every plotted signal also remains available for one variant.
            diagnostics = ["nominal_c1_trajectory_3d", ...
                "nominal_c1_position", "nominal_c1_wrench_force", ...
                "euclidean_c1_parameter_error", ...
                "euclidean_c1_pseudo_inertia_margin", ...
                "bregman_c2_inertia_principal", ...
                "bregman_c2_pseudo_inertia_margin"];
            testCase.verifyTrue(all(ismember(diagnostics, names)));

            nominalDiagnostic = output(names == "nominal_c1_position");
            testCase.verifyEqual(nominalDiagnostic.directory, ...
                fullfile(suiteDirectory, 'nominal_c1', 'figures'));
            comparison = output(names == "nominal_position_error_c1_c2");
            testCase.verifyEqual(comparison.directory, ...
                fullfile(suiteDirectory, 'comparisons'));

            for item = output
                testCase.verifyTrue(isfile(item.files.png));
                testCase.verifyTrue(isfile(item.files.pdf));
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
