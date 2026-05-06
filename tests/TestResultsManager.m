classdef TestResultsManager < matlab.unittest.TestCase
    %TESTRESULTSMANAGER Unit tests for result persistence helper behavior.

    methods (Test)
        function testBuildRunNameSingleAndBatch(testCase)
            cfg = vt.config.Config();
            cfg.setTrajectory('circle');
            cfg.setController('Feedforward', 'liealgebra');

            singleName = vt.sim.ResultsManager.buildRunName(cfg, false);
            batchName = vt.sim.ResultsManager.buildRunName(cfg, true);

            testCase.verifyTrue(~isempty(regexp(singleName, '^\d{8}_\d{6}_circle_ff_lie$', 'once')));
            testCase.verifyTrue(~isempty(regexp(batchName, '^\d{8}_\d{6}_circle$', 'once')));
        end

        function testWriteAndReadTextFileRoundtrip(testCase)
            rootDir = tempname;
            mkdir(rootDir);
            c = onCleanup(@() rmdir(rootDir, 's'));
            filePath = fullfile(rootDir, 'notes.txt');

            vt.sim.ResultsManager.writeTextFile(filePath, sprintf('hello%sworld', newline));
            content = vt.sim.ResultsManager.readTextFile(filePath);
            testCase.verifyTrue(contains(content, 'hello'));
            testCase.verifyTrue(contains(content, 'world'));
        end

        function testPersistAndLoadRunRoundtrip(testCase)
            rootDir = tempname;
            mkdir(rootDir);
            c = onCleanup(@() rmdir(rootDir, 's'));

            logs = struct('actual', struct('pos', [0 0 0]), 'des', struct('pos', [0 0 0]));
            metrics = struct('combined', struct('rmse_total', 0.1, 'tracking_score', 99));
            est = struct('mass', 3.5);
            runInfo = struct('isAdaptive', true, 'duration', 1, 'dt', 0.01);
            cfg = vt.config.Config();

            vt.sim.ResultsManager.persistRun(rootDir, logs, metrics, est, runInfo, cfg);
            saved = vt.sim.ResultsManager.loadRun(rootDir);

            testCase.verifyEqual(saved.metrics.combined.rmse_total, 0.1);
            testCase.verifyEqual(saved.est.mass, 3.5);
            testCase.verifyEqual(saved.runInfo.isAdaptive, true);
            testCase.verifyEqual(saved.cfgSnapshot.traj.name, cfg.traj.name);
        end

        function testLoadRunMissingDataThrows(testCase)
            rootDir = tempname;
            mkdir(rootDir);
            c = onCleanup(@() rmdir(rootDir, 's'));
            testCase.verifyError(@() vt.sim.ResultsManager.loadRun(rootDir), ...
                'ResultsManager:MissingSavedData');
        end

        function testFindChildResultDirsReturnsSortedRunDirs(testCase)
            rootDir = tempname;
            mkdir(rootDir);
            c = onCleanup(@() rmdir(rootDir, 's'));

            run1 = fullfile(rootDir, 't02_inf3d', 'run_002');
            run2 = fullfile(rootDir, 't01_circle', 'run_001');
            mkdir(run1);
            mkdir(run2);
            save(fullfile(run1, 'sim_data.mat'), 'rootDir');
            save(fullfile(run2, 'sim_data.mat'), 'rootDir');

            dirs = vt.sim.ResultsManager.findChildResultDirs(rootDir);
            testCase.verifyEqual(dirs, sort({run1; run2}));
        end

        function testFindChildResultDirsSupportsMetricsOnlyRuns(testCase)
            rootDir = tempname;
            mkdir(rootDir);
            c = onCleanup(@() rmdir(rootDir, 's'));

            run1 = fullfile(rootDir, 't02_inf3d', 'run_002');
            run2 = fullfile(rootDir, 't01_circle', 'run_001');
            mkdir(run1);
            mkdir(run2);
            vt.sim.ResultsManager.writeTextFile(fullfile(run1, 'metrics.txt'), 'trajectory=infinity3d');
            vt.sim.ResultsManager.writeTextFile(fullfile(run2, 'metrics.txt'), 'trajectory=circle');

            dirs = vt.sim.ResultsManager.findChildResultDirs(rootDir);
            testCase.verifyEqual(dirs, sort({run1; run2}));
        end

        function testCreateResultsDirUsesAdaptiveSubfolder(testCase)
            repoRoot = tempname;
            mkdir(repoRoot);
            c = onCleanup(@() rmdir(repoRoot, 's'));

            cfg = vt.config.Config();
            cfg.setAdaptation('euclidean');
            runDir = vt.sim.ResultsManager.createResultsDir(cfg, repoRoot, 'sample_run');
            testCase.verifyTrue(exist(runDir, 'dir') == 7);
            testCase.verifyTrue(contains(runDir, fullfile('results', 'adaptive', 'sample_run')));
        end

        function testPlotSavedRunSummaryHiddenByDefault(testCase)
            rootDir = tempname;
            mkdir(rootDir);
            c = onCleanup(@() rmdir(rootDir, 's'));
            [cfg, logs, est, runInfo, metrics] = testCase.buildSavedRunFixture(false);
            vt.sim.ResultsManager.persistRun(rootDir, logs, metrics, est, runInfo, cfg);

            beforeFigures = findall(groot, 'Type', 'figure');
            vt.sim.ResultsManager.plotSavedRun(rootDir, 'summary');
            afterFigures = findall(groot, 'Type', 'figure');

            testCase.verifyEqual(numel(afterFigures), numel(beforeFigures));
            testCase.verifyTrue(exist(fullfile(rootDir, 'summary_nominal.png'), 'file') == 2);
        end

        function testPlotSavedRunAcceptsDirectSimDataPath(testCase)
            rootDir = tempname;
            mkdir(rootDir);
            c = onCleanup(@() rmdir(rootDir, 's'));
            [cfg, logs, est, runInfo, metrics] = testCase.buildSavedRunFixture(false);
            vt.sim.ResultsManager.persistRun(rootDir, logs, metrics, est, runInfo, cfg);

            simDataPath = fullfile(rootDir, 'sim_data.mat');
            vt.sim.ResultsManager.plotSavedRun(simDataPath, 'summary', false);

            testCase.verifyTrue(exist(fullfile(rootDir, 'summary_nominal.png'), 'file') == 2);
        end

        function testPlotSavedRunAllHiddenCreatesExpectedAdaptivePngs(testCase)
            rootDir = tempname;
            mkdir(rootDir);
            c = onCleanup(@() rmdir(rootDir, 's'));
            [cfg, logs, est, runInfo, metrics] = testCase.buildSavedRunFixture(true);
            vt.sim.ResultsManager.persistRun(rootDir, logs, metrics, est, runInfo, cfg);

            beforeFigures = findall(groot, 'Type', 'figure');
            vt.sim.ResultsManager.plotSavedRun(rootDir, 'all', false);
            afterFigures = findall(groot, 'Type', 'figure');

            testCase.verifyEqual(numel(afterFigures), numel(beforeFigures));
            testCase.verifyTrue(exist(fullfile(rootDir, 'summary_adaptive.png'), 'file') == 2);
            testCase.verifyTrue(exist(fullfile(rootDir, 'standalone_mass_cog.png'), 'file') == 2);
            testCase.verifyTrue(exist(fullfile(rootDir, 'stack_estimation.png'), 'file') == 2);
            testCase.verifyTrue(exist(fullfile(rootDir, 'stack_wrench.png'), 'file') == 2);
        end

        function testPlotSavedRunNoneSkipsFigureGeneration(testCase)
            rootDir = tempname;
            mkdir(rootDir);
            c = onCleanup(@() rmdir(rootDir, 's'));
            [cfg, logs, est, runInfo, metrics] = testCase.buildSavedRunFixture(false);
            vt.sim.ResultsManager.persistRun(rootDir, logs, metrics, est, runInfo, cfg);

            vt.sim.ResultsManager.plotSavedRun(rootDir, 'none', false);

            testCase.verifyFalse(exist(fullfile(rootDir, 'summary_nominal.png'), 'file') == 2);
        end

        function testWriteAndLoadMetricsFileRoundtrip(testCase)
            rootDir = tempname;
            mkdir(rootDir);
            c = onCleanup(@() rmdir(rootDir, 's'));
            [cfg, ~, ~, runInfo, metrics] = testCase.buildSavedRunFixture(true);
            cfg.sim.batchRunIndex = 3;

            vt.sim.ResultsManager.writeMetricsFile(rootDir, metrics, runInfo, cfg);
            entry = vt.sim.ResultsManager.loadMetricsFile(rootDir);

            testCase.verifyEqual(entry.trajectory, 'circle');
            testCase.verifyEqual(entry.run_label, 'Run 3');
            testCase.verifyEqual(entry.is_adaptive, true);
            testCase.verifyEqual(entry.track_rmse, metrics.combined.rmse_total, 'AbsTol', 1e-12);
            testCase.verifyEqual(entry.cog_tracking_score, metrics.parameters.cog.tracking_score, 'AbsTol', 1e-12);
            testCase.verifyEqual(entry.mass_ident_score, metrics.parameters.identifiability.mass.score, 'AbsTol', 1e-12);
            testCase.verifyEqual(entry.mass_nrmse, metrics.parameters.mass.nrmse, 'AbsTol', 1e-12);
            testCase.verifyEqual(entry.cog_ident_metric, metrics.parameters.identifiability.mcog.sigma_min, 'AbsTol', 1e-12);
        end

        function testAdaptiveRunWithoutPayloadBuildsMetricsAndNoShapeError(testCase)
            startup;
            cfg = vt.config.Config();
            cfg.setController('Feedforward');
            cfg.setPotentialType('liealgebra');
            cfg.setAdaptation('euclidean');
            cfg.setTrajectory('hover', 1, true);
            cfg.setSimParams(0.005, 0.02);
            cfg.setAdaptationParams(0.005);
            cfg.setControlParams(0.01);
            cfg.setAdaptiveGains(1e-2 * [8 8 12 0.4 0.4 0.4 36 12 12 12]);
            cfg.enableLiveView(false);
            cfg.done();

            sim = vt.sim.SimRunner(cfg);
            sim.setup();
            sim.run('none', false, false);

            testCase.verifyTrue(exist(fullfile(sim.resultsDir, 'metrics.txt'), 'file') == 2);
            metricsEntry = vt.sim.ResultsManager.loadMetricsFile(sim.resultsDir);
            testCase.verifyEqual(metricsEntry.is_adaptive, true);
            testCase.verifyTrue(isfinite(metricsEntry.cog_rmse));
            testCase.verifyTrue(isfinite(metricsEntry.mass_nrmse));
            testCase.verifyTrue(isfinite(metricsEntry.mass_ident_score));
            testCase.verifyTrue(isfinite(metricsEntry.cog_ident_score));
            testCase.verifyTrue(isfinite(metricsEntry.inertia_ident_score));
        end

        function testAdaptiveRunWithPayloadDropBuildsMetricsAndNoShapeError(testCase)
            startup;
            cfg = vt.config.Config();
            cfg.setController('Feedforward');
            cfg.setPotentialType('liealgebra');
            cfg.setAdaptation('euclidean');
            cfg.setTrajectory('hover', 1, true);
            cfg.setSimParams(0.005, 0.02);
            cfg.setAdaptationParams(0.005);
            cfg.setControlParams(0.01);
            cfg.setAdaptiveGains(1e-2 * [8 8 12 0.4 0.4 0.4 36 12 12 12]);
            cfg.setPayloadScenario(0.5, [0.01; 0; -0.02], 0.01);
            cfg.enableLiveView(false);
            cfg.done();

            sim = vt.sim.SimRunner(cfg);
            sim.setup();
            sim.run(true, 0.5, [0.01; 0; -0.02], 0.01, 'nominal', 'none', false, false);

            testCase.verifyTrue(exist(fullfile(sim.resultsDir, 'metrics.txt'), 'file') == 2);
            metricsEntry = vt.sim.ResultsManager.loadMetricsFile(sim.resultsDir);
            testCase.verifyEqual(metricsEntry.is_adaptive, true);
            testCase.verifyTrue(isfinite(metricsEntry.cog_rmse));
            testCase.verifyTrue(isfinite(metricsEntry.mass_nrmse));
            testCase.verifyTrue(isfinite(metricsEntry.mass_ident_score));
            testCase.verifyTrue(isfinite(metricsEntry.cog_ident_score));
            testCase.verifyTrue(isfinite(metricsEntry.inertia_ident_score));
        end

        function testAdaptiveBatchWritesIdentReportAndExcludesZeroGains(testCase)
            startup;
            rootDir = tempname;
            mkdir(rootDir);
            c = onCleanup(@() rmdir(rootDir, 's'));

            cfg = vt.config.Config();
            cfg.setController('Feedforward');
            cfg.setPotentialType('liealgebra');
            cfg.setAdaptation('euclidean');
            cfg.setTrajectory({'hover', 'circle'}, [1 1], [true true]);
            cfg.setSimParams(0.005, 0.02);
            cfg.setAdaptationParams(0.005);
            cfg.setControlParams(0.01);
            cfg.setAdaptiveGains([zeros(1,10); 1e-2 * [8 8 12 0.4 0.4 0.4 36 12 12 12]]);
            cfg.enableLiveView(false);
            cfg.sim.resultsDirOverride = rootDir;
            cfg.done();

            sim = vt.sim.SimRunner(cfg);
            sim.setup();
            sim.run('none', false, false);

            identPath = fullfile(sim.resultsDir, 'ident_report.txt');
            testCase.verifyTrue(exist(identPath, 'file') == 2);
            content = vt.sim.ResultsManager.readTextFile(identPath);
            testCase.verifyTrue(contains(content, 'Batch Identifiability Report'));
            testCase.verifyTrue(contains(content, 'Gain 002'));
            testCase.verifyFalse(contains(content, 'Gain 001'));
            testCase.verifyTrue(contains(content, 'Trajectory Mean Summary'));
            testCase.verifyTrue(contains(content, 'hover'));
            testCase.verifyTrue(contains(content, 'circle'));
        end
    end

    methods (Access = private)
        function [cfg, logs, est, runInfo, metrics] = buildSavedRunFixture(~, isAdaptive)
            t = [0; 1; 2];
            posAct = [0 0 0; 0.5 0.2 0.8; 1.0 0.4 1.2];
            posDes = [0 0 0; 0.4 0.3 0.9; 1.1 0.5 1.3];
            rpyAct = [0 0 0; 0.02 0.01 0.03; 0.04 0.02 0.05];
            rpyDes = [0 0 0; 0.01 0.015 0.025; 0.03 0.025 0.04];
            linVelAct = [0 0 0; 0.5 0.2 0.4; 0.4 0.1 0.2];
            linVelDes = [0 0 0; 0.4 0.25 0.45; 0.35 0.15 0.25];
            angVelAct = [0 0 0; 0.05 0.02 0.01; 0.04 0.03 0.02];
            angVelDes = [0 0 0; 0.04 0.03 0.01; 0.03 0.02 0.015];
            wrenchF = [0 0 9; 0.5 0.1 9.2; 0.4 0.0 9.1];
            wrenchT = [0 0 0; 0.01 0.02 0.03; 0.02 0.01 0.02];

            logs = struct( ...
                't', t, ...
                'actual', struct('pos', posAct, 'rpy', rpyAct, 'linVel', linVelAct, 'angVel', angVelAct), ...
                'des', struct('pos', posDes, 'rpy', rpyDes, 'linVel', linVelDes, 'angVel', angVelDes), ...
                'cmd', struct('wrenchF', wrenchF, 'wrenchT', wrenchT));

            est = [];
            metrics = struct('combined', struct('rmse_total', 0.12, 'tracking_score', 98.7));
            cfg = vt.config.Config();
            cfg.setTrajectory('circle');
            if isAdaptive
                cfg.setAdaptation('euclidean');
                est = struct( ...
                    't', t, ...
                    'mass', [4.1; 4.3; 4.2], ...
                    'com', [0.01 0.00 -0.01; 0.02 0.01 -0.01; 0.03 0.01 0.00], ...
                    'inertia', [ ...
                        0.05 0.05 0.07 0.001 0.002 0.003; ...
                        0.051 0.052 0.071 0.0015 0.0025 0.0035; ...
                        0.052 0.053 0.072 0.0020 0.0030 0.0040], ...
                    'massActual', [5.0; 5.0; 3.646], ...
                    'comActual', [0.02 0.01 -0.01; 0.02 0.01 -0.01; 0 0 -0.00229], ...
                    'inertiaActual', [ ...
                        0.06 0.07 0.08 0.004 0.005 0.006; ...
                        0.06 0.07 0.08 0.004 0.005 0.006; ...
                        0.04092 0.04017 0.06921 5.656e-5 -6.494e-5 1.313e-5], ...
                    'dropTime', 1.5);
                metrics.parameters = struct( ...
                    'mass', struct('rmse', 0.2, 'nrmse', 0.04, 'tracking_score', 85.0), ...
                    'cog', struct('rmse_total', 0.01, 'nrmse_total', 0.08, 'tracking_score', 75.0), ...
                    'inertia', struct('rmse_total', 0.02, 'nrmse_total', 0.12, 'tracking_score', 65.0), ...
                    'identifiability', struct( ...
                        'mass', struct('score', 91.0, 'sigma_min', 0.91), ...
                        'mcog', struct('score', 73.0, 'sigma_min', 0.73), ...
                        'inertia', struct('score', 62.0, 'sigma_min', 0.62)));
            end

            runInfo = struct('isAdaptive', isAdaptive, 'duration', 2, 'dt', 1, ...
                'control_dt', 1, 'adaptation_dt', 1, 'runName', 'fixture_run');
        end
    end
end
