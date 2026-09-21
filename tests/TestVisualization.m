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
            testCase.verifyEqual(string({nominalDiagnostic.directory}), [ ...
                string(fullfile(suiteDirectory, 'nominal_c1', 'figures', 'total-sim')), ...
                string(fullfile(suiteDirectory, 'nominal_c1', 'figures', 'from-drop'))]);
            comparison = output(names == "nominal_position_error_c1_c2");
            testCase.verifyEqual(string({comparison.directory}), [ ...
                string(fullfile(suiteDirectory, 'comparisons', 'total-sim', 'nominal')), ...
                string(fullfile(suiteDirectory, 'comparisons', 'from-drop', 'nominal'))]);
            adaptiveComparison = output(names == "euclidean_vs_bregman_position_error_c1");
            testCase.verifyEqual(string({adaptiveComparison.directory}), [ ...
                string(fullfile(suiteDirectory, 'comparisons', 'total-sim', 'euclidean_v_bregman')), ...
                string(fullfile(suiteDirectory, 'comparisons', 'from-drop', 'euclidean_v_bregman'))]);
            performance = output(names == "performance_position_rmse");
            testCase.verifyEqual(performance.directory, ...
                fullfile(suiteDirectory, 'comparisons', 'total-sim', 'performance'));

            for item = output
                testCase.verifyTrue(isfile(item.files.png));
                testCase.verifyTrue(isfile(item.files.pdf));
            end
        end

        function paperFiguresExportsSuccessfulMembersOfPartialSuite(testCase)
            suiteDirectory = tempname;
            cleanup = onCleanup(@() removeDirectory(suiteDirectory)); %#ok<NASGU>
            variants = {'euclidean', 'c2'; 'bregman', 'c2'};

            for k = 1:size(variants, 1)
                resultDirectory = fullfile(suiteDirectory, sprintf('%s_%s', variants{k,:}));
                [scenario, run, metrics] = fixture(variants{k,1}, variants{k,2});
                agc.io.saveRun(resultDirectory, scenario, run, metrics);
            end

            output = agc.viz.paperFigures(suiteDirectory, struct('visible', false));
            names = string({output.name});

            testCase.verifyTrue(any(names == "euclidean_c2_position"));
            testCase.verifyTrue(any(names == "bregman_c2_position"));
            testCase.verifyTrue(any(names == "euclidean_vs_bregman_position_error_c2"));
            testCase.verifyTrue(any(names == "performance_position_rmse"));
            testCase.verifyFalse(any(contains(names, "_c1_c2")));
        end

        function paperFiguresExportFinitePrefixFromFailedRun(testCase)
            suiteDirectory = tempname;
            cleanup = onCleanup(@() removeDirectory(suiteDirectory)); %#ok<NASGU>
            resultDirectory = fullfile(suiteDirectory, 'nominal_c2');
            [scenario, run, metrics] = fixture('nominal', 'c2');
            run = truncateRun(run, 2);
            failure = struct('identifier', 'agc:test:failure', 'message', 'failed after release');
            agc.io.saveRun(resultDirectory, scenario, run, metrics, failure);

            output = agc.viz.paperFigures(suiteDirectory, ...
                struct('visible', false, 'exportComparisons', false));
            names = string({output.name});
            position = output(names == "nominal_c2_position");

            testCase.verifyEqual(string({position.directory}), [ ...
                string(fullfile(resultDirectory, 'figures', 'total-sim')), ...
                string(fullfile(resultDirectory, 'figures', 'from-drop'))]);
        end

        function paperFiguresCanPreserveExistingComparisonDirectory(testCase)
            suiteDirectory = tempname;
            cleanup = onCleanup(@() removeDirectory(suiteDirectory)); %#ok<NASGU>
            resultDirectory = fullfile(suiteDirectory, 'nominal_c2');
            [scenario, run, metrics] = fixture('nominal', 'c2');
            agc.io.saveRun(resultDirectory, scenario, run, metrics);

            comparisonFile = fullfile(suiteDirectory, 'comparisons', 'keep.txt');
            mkdir(fileparts(comparisonFile));
            fid = fopen(comparisonFile, 'w'); fprintf(fid, 'preserve'); fclose(fid);

            output = agc.viz.paperFigures(suiteDirectory, ...
                struct('visible', false, 'exportComparisons', false));

            testCase.verifyTrue(isfile(comparisonFile));
            testCase.verifyTrue(any(string({output.name}) == "nominal_c2_position"));
            testCase.verifyFalse(any(contains(string({output.directory}), "comparisons")));
        end

        function traceHelpersBoundContinuousSamplesAndPreserveZohChanges(testCase)
            time = (0:0.002:30).';
            signal = [sin(time), cos(time)];
            wrench = zeros(numel(time), 1);
            wrench(time >= 10) = 2;
            wrench(time >= 20) = -1;

            [sampleTime, sampleSignal] = agc.viz.decimateTrace(time, signal, 2000);
            [stepTime, stepWrench] = agc.viz.zohTrace(time, wrench);
            logSignal = agc.viz.logTrace([1; 0; 1e-14; -1]);

            testCase.verifyLessThanOrEqual(numel(sampleTime), 2000);
            testCase.verifyEqual(sampleTime([1, end]), time([1, end]));
            testCase.verifyEqual(sampleSignal([1, end],:), signal([1, end],:));
            testCase.verifyEqual(stepTime, [0; 10; 20; 30], 'AbsTol', 1e-12);
            testCase.verifyEqual(stepWrench, [0; 2; -1; -1]);
            testCase.verifyTrue(isnan(logSignal(2)));
            testCase.verifyTrue(isnan(logSignal(4)));
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
payload = struct('mass', 0.2, 'dimensions', [0.1; 0.08; 0.06], ...
    'center', [0.1; 0; -0.05]);
loadedPi = agc.plant.compoundPi(pi, payload);
activePlantPi = repmat(pi.', n, 1);
activePlantPi(t < 0.1,:) = repmat(loadedPi.', sum(t < 0.1), 1);
estimatePi = repmat(loadedPi.', n, 1);
if ~strcmpi(mode, 'nominal')
    estimatePi = estimatePi .* (1 + 0.02 * exp(-t));
end
run = struct('t', t, 'H', H, 'V', zeros(n,6), 'Hdesired', Hd, 'Vdesired', zeros(n,6), ...
    'wrench', repmat([0, 0, 0, 0, 0, 19.62], n, 1), 's', repmat([0, 0, 0, 0.1, 0, 0], n, 1), ...
    'Psi', exp(-t), 'Vs', 0.1 * exp(-t), 'estimatePi', estimatePi, ...
    'activePlantPi', activePlantPi, ...
    'minPseudoEigenvalue', ternary(strcmpi(mode, 'bregman'), 0.01 * ones(n,1), nan(n,1)), ...
    'mode', mode, 'coriolis', coriolis, 'finalEstimate', pi);
scenario = struct('plantPi', pi, 'payloadDrop', struct('releaseTime', 0.1, ...
    'barePi', pi, 'loadedPi', loadedPi, 'payload', payload));
metrics = agc.sim.metrics(run);
end

function value = ternary(condition, trueValue, falseValue)
if condition, value = trueValue; else, value = falseValue; end
end

function run = truncateRun(run, count)
run.t = run.t(1:count);
run.H = run.H(:,:,1:count);
run.V = run.V(1:count,:);
run.Hdesired = run.Hdesired(:,:,1:count);
run.Vdesired = run.Vdesired(1:count,:);
run.wrench = run.wrench(1:count,:);
run.s = run.s(1:count,:);
run.Psi = run.Psi(1:count);
run.Vs = run.Vs(1:count);
run.estimatePi = run.estimatePi(1:count,:);
run.activePlantPi = run.activePlantPi(1:count,:);
run.minPseudoEigenvalue = run.minPseudoEigenvalue(1:count);
end

function removeDirectory(directory)
if isfolder(directory), rmdir(directory, 's'); end
end
