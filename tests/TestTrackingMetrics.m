classdef TestTrackingMetrics < matlab.unittest.TestCase
    %TESTTRACKINGMETRICS Unit tests for TrackingMetrics with known error values.

    methods (Test)
        function testZeroErrorGivesPerfectScore(testCase)
            %Zero tracking error should yield RMSE=0 and score=100%.
            N = 100;
            pos = repmat([1 2 3], N, 1);
            rpy = repmat([0.1 0.2 0.3], N, 1);
            logs = testCase.buildLogs(pos, pos, rpy, rpy);
            tm = vt.metrics.TrackingMetrics(logs, 'ZeroError');
            m = tm.computeAll();
            testCase.verifyEqual(m.position.rmse_total, 0, 'AbsTol', 1e-14);
            testCase.verifyEqual(m.position.tracking_score, 100, 'AbsTol', 1e-10);
            testCase.verifyEqual(m.orientation.rmse_total, 0, 'AbsTol', 1e-12);
            testCase.verifyEqual(m.combined.rmse_total, 0, 'AbsTol', 1e-12);
        end

        function testConstantPositionOffset(testCase)
            %Constant offset of [1,0,0] should give RMSE=1 in position.
            N = 200;
            desPos = zeros(N, 3);
            actPos = repmat([1 0 0], N, 1);
            rpy = zeros(N, 3);
            logs = testCase.buildLogs(actPos, desPos, rpy, rpy);
            tm = vt.metrics.TrackingMetrics(logs, 'ConstOffset');
            m = tm.computePosition();
            testCase.verifyEqual(m.rmse_total, 1.0, 'AbsTol', 1e-14);
            testCase.verifyEqual(m.rmse_xyz, [1 0 0], 'AbsTol', 1e-14);
            testCase.verifyEqual(m.max_error, 1.0, 'AbsTol', 1e-14);
            testCase.verifyEqual(m.mean_error, 1.0, 'AbsTol', 1e-14);
            testCase.verifyEqual(m.std_error, 0, 'AbsTol', 1e-14);
        end

        function testKnownRMSE(testCase)
            %Known sinusoidal error should give predictable RMSE.
            N = 10000;
            t = linspace(0, 2*pi, N)';
            desPos = zeros(N, 3);
            actPos = [sin(t), zeros(N, 1), zeros(N, 1)];
            rpy = zeros(N, 3);
            logs = testCase.buildLogs(actPos, desPos, rpy, rpy);
            tm = vt.metrics.TrackingMetrics(logs, 'Sinusoidal');
            m = tm.computePosition();
            % RMS of sin(t) over a full period is 1/sqrt(2)
            expected_rmse = 1 / sqrt(2);
            testCase.verifyEqual(m.rmse_total, expected_rmse, 'AbsTol', 1e-3);
        end

        function testOrientationZeroError(testCase)
            N = 50;
            rpy = repmat([0.1 -0.2 0.5], N, 1);
            logs = testCase.buildLogs(zeros(N,3), zeros(N,3), rpy, rpy);
            tm = vt.metrics.TrackingMetrics(logs, 'OriZero');
            m = tm.computeOrientation();
            testCase.verifyEqual(m.rmse_total, 0, 'AbsTol', 1e-12);
        end

        function testCombinedScoreBetweenZeroAndHundred(testCase)
            %Combined tracking score must always be in [0, 100].
            N = 100;
            desPos = repmat([0 0 5], N, 1);
            actPos = desPos + 0.1 * randn(N, 3);
            rpy = zeros(N, 3);
            desRpy = rpy;
            logs = testCase.buildLogs(actPos, desPos, rpy, desRpy);
            tm = vt.metrics.TrackingMetrics(logs, 'Bounded');
            m = tm.computeCombined();
            testCase.verifyGreaterThanOrEqual(m.tracking_score, 0);
            testCase.verifyLessThanOrEqual(m.tracking_score, 100);
        end

        function testPositionMetricsCaching(testCase)
            %Calling computePosition twice should return the same result.
            N = 50;
            desPos = zeros(N, 3);
            actPos = repmat([0.5 0.5 0.5], N, 1);
            rpy = zeros(N, 3);
            logs = testCase.buildLogs(actPos, desPos, rpy, rpy);
            tm = vt.metrics.TrackingMetrics(logs, 'Cache');
            m1 = tm.computePosition();
            m2 = tm.computePosition();
            testCase.verifyEqual(m1.rmse_total, m2.rmse_total);
        end

        function testParameterEstimationMassRMSE(testCase)
            %Known mass estimation error.
            N = 100;
            estMass = 4.0 * ones(N, 1);
            actualMass = 3.646 * ones(N, 1);
            expectedRMSE = abs(4.0 - 3.646);
            logs = testCase.buildAdaptiveLogs(N, estMass, actualMass, ...
                zeros(N, 3), zeros(N, 3), zeros(N, 6), zeros(N, 6));
            tm = vt.metrics.TrackingMetrics(logs, 'MassRMSE');
            m = tm.computeAll();
            testCase.verifyEqual(m.parameters.mass.rmse, expectedRMSE, 'AbsTol', 1e-12);
        end

        function testParameterEstimationPerfectMass(testCase)
            %Perfect mass estimation should give score=100.
            N = 100;
            mass = 3.646 * ones(N, 1);
            logs = testCase.buildAdaptiveLogs(N, mass, mass, ...
                zeros(N, 3), zeros(N, 3), zeros(N, 6), zeros(N, 6));
            tm = vt.metrics.TrackingMetrics(logs, 'PerfectMass');
            m = tm.computeAll();
            testCase.verifyEqual(m.parameters.mass.rmse, 0, 'AbsTol', 1e-14);
        end

        function testIdentifiabilityScoresHighForIndependentGroups(testCase)
            N = 5;
            F = eye(10);
            ident = struct('infoMatrix', F, 'updateCount', N);
            logs = testCase.buildAdaptiveLogs(N, ones(N, 1), ones(N, 1), ...
                zeros(N, 3), zeros(N, 3), zeros(N, 6), zeros(N, 6), ident);
            tm = vt.metrics.TrackingMetrics(logs, 'IdentHigh');
            m = tm.computeAll();

            testCase.verifyEqual(m.parameters.identifiability.mass.score, 100, 'AbsTol', 1e-12);
            testCase.verifyEqual(m.parameters.identifiability.mcog.score, 100, 'AbsTol', 1e-12);
            testCase.verifyEqual(m.parameters.identifiability.inertia.score, 100, 'AbsTol', 1e-12);
            testCase.verifyEqual(m.parameters.identifiability.mass.sigma_min, 1, 'AbsTol', 1e-12);
            testCase.verifyEqual(m.parameters.identifiability.update_count, N);
        end

        function testIdentifiabilityMassScoreLowWhenExplainedByOthers(testCase)
            N = 1;
            F = eye(10);
            F(7, 7) = 1;
            F(1, 7) = 1;
            F(7, 1) = 1;
            ident = struct('infoMatrix', F, 'updateCount', N);
            logs = testCase.buildAdaptiveLogs(N, ones(N, 1), ones(N, 1), ...
                zeros(N, 3), zeros(N, 3), zeros(N, 6), zeros(N, 6), ident);
            tm = vt.metrics.TrackingMetrics(logs, 'IdentMassLow');
            m = tm.computeAll();

            testCase.verifyLessThan(m.parameters.identifiability.mass.score, 1e-9);
        end

        function testIdentifiabilityMcogLowWhenOneDirectionUnexcited(testCase)
            N = 1;
            F = eye(10);
            F(10, 10) = 0;
            ident = struct('infoMatrix', F, 'updateCount', N);
            logs = testCase.buildAdaptiveLogs(N, ones(N, 1), ones(N, 1), ...
                zeros(N, 3), zeros(N, 3), zeros(N, 6), zeros(N, 6), ident);
            tm = vt.metrics.TrackingMetrics(logs, 'IdentMcogLow');
            m = tm.computeAll();

            testCase.verifyEqual(m.parameters.identifiability.mcog.rank, 2);
            testCase.verifyLessThan(m.parameters.identifiability.mcog.score, 1e-9);
        end

        function testIdentifiabilityDefaultsToNaNWhenUnavailable(testCase)
            N = 10;
            logs = testCase.buildAdaptiveLogs(N, ones(N, 1), ones(N, 1), ...
                zeros(N, 3), zeros(N, 3), zeros(N, 6), zeros(N, 6));
            tm = vt.metrics.TrackingMetrics(logs, 'IdentMissing');
            m = tm.computeAll();

            testCase.verifyTrue(isnan(m.parameters.identifiability.mass.score));
            testCase.verifyTrue(isnan(m.parameters.identifiability.mcog.score));
            testCase.verifyTrue(isnan(m.parameters.identifiability.inertia.score));
        end

        function testMissingLogsThrows(testCase)
            tm = vt.metrics.TrackingMetrics();
            testCase.verifyError(@() tm.computeAll(), 'TrackingMetrics:MissingLogs');
        end

        function testComputeAllHasExpectedFields(testCase)
            N = 50;
            logs = testCase.buildLogs(zeros(N,3), zeros(N,3), zeros(N,3), zeros(N,3));
            tm = vt.metrics.TrackingMetrics(logs, 'Fields');
            m = tm.computeAll();
            testCase.verifyTrue(isfield(m, 'position'));
            testCase.verifyTrue(isfield(m, 'orientation'));
            testCase.verifyTrue(isfield(m, 'combined'));
            testCase.verifyTrue(isfield(m.position, 'rmse_total'));
            testCase.verifyTrue(isfield(m.position, 'tracking_score'));
            testCase.verifyTrue(isfield(m.combined, 'tracking_score'));
        end
    end

    methods (Static, Access = private)
        function logs = buildLogs(actPos, desPos, actRpy, desRpy)
            %BUILDLOGS Create a minimal logs struct for testing.
            logs = struct();
            logs.actual.pos = actPos;
            logs.des.pos = desPos;
            logs.actual.rpy = actRpy;
            logs.des.rpy = desRpy;
        end

        function logs = buildAdaptiveLogs(N, estMass, actualMass, estCoG, actualCoG, estInertia, actualInertia, ident)
            %BUILDADAPTIVELOGS Create logs with estimation data for adaptive tests.
            if nargin < 8
                ident = [];
            end
            logs = struct();
            logs.actual.pos = zeros(N, 3);
            logs.des.pos = zeros(N, 3);
            logs.actual.rpy = zeros(N, 3);
            logs.des.rpy = zeros(N, 3);
            logs.est.mass = estMass;
            logs.est.massActual = actualMass;
            logs.est.com = estCoG;
            logs.est.comActual = actualCoG;
            logs.est.inertia = estInertia;
            logs.est.inertiaActual = actualInertia;
            if ~isempty(ident)
                logs.est.identifiability = ident;
            end
        end
    end
end
