classdef TestReplayWnojSmoother < matlab.unittest.TestCase
    methods (Test)
        function testUsesInstanceApi(testCase)
            t = (0:0.02:0.10).';
            R = repmat(eye(3), 1, 1, numel(t));
            p = [t, zeros(numel(t), 2)];
            VBody = repmat([0, 0, 0, 1, 0, 0], numel(t), 1);

            smoother = ReplayWnojSmoother(struct('knotIntervalSeconds', 0.02));
            smoother = smoother.fit(t, R, p, VBody);
            [H, V, A] = smoother.evaluate(0.04);

            testCase.verifyClass(smoother, 'ReplayWnojSmoother');
            testCase.verifyEqual(size(H), [4 4]);
            testCase.verifyEqual(size(V), [6 1]);
            testCase.verifyEqual(size(A), [6 1]);
            testCase.verifyEqual(smoother.outputTimes, t, 'AbsTol', 1e-12);
            testCase.verifyTrue(smoother.diagnostics.converged);
        end

        function testWnojTransitionMatchesClosedForm(testCase)
            dt = 0.02;
            Qc = diag([1; 2; 3; 4; 5; 6]);
            [Phi, Q] = ReplayWnojSmoother.transition(dt, Qc);

            I = eye(6);
            expectedPhi = [I, dt * I, 0.5 * dt^2 * I; ...
                zeros(6), I, dt * I; zeros(6), zeros(6), I];
            expectedQ = [dt^5 / 20 * Qc, dt^4 / 8 * Qc, dt^3 / 6 * Qc; ...
                dt^4 / 8 * Qc, dt^3 / 3 * Qc, dt^2 / 2 * Qc; ...
                dt^3 / 6 * Qc, dt^2 / 2 * Qc, dt * Qc];

            testCase.verifyEqual(Phi, expectedPhi, 'AbsTol', 1e-14);
            testCase.verifyEqual(Q, expectedQ, 'AbsTol', 1e-14);
        end

        function testWnojPriorResidualUsesBodyStateAndAcceleration(testCase)
            dt = 0.1;
            Ti = eye(4);
            Vi = [0.1; -0.2; 0.05; 1.0; 0.2; -0.1];
            Ai = [0.03; -0.02; 0.01; 0.4; -0.1; 0.2];
            Tj = fth.se3.expSE3(fth.se3.vec2tilde( ...
                dt * Vi + 0.5 * dt^2 * Ai)) * Ti;
            Vj = Vi + dt * Ai;
            Aj = Ai;

            e = ReplayWnojSmoother.priorResidual( ...
                Ti, Vi, Ai, Tj, Vj, Aj, dt);

            testCase.verifyEqual(size(e), [18 1]);
            testCase.verifyLessThan(norm(e), 0.1);
        end

        function testFitsAcceleratedSE3Trajectory(testCase)
            t = (0:0.01:0.20).';
            acceleration = [0.4; -0.2; 0.1];
            p = 0.5 * (t.^2) * acceleration.';
            R = repmat(eye(3), 1, 1, numel(t));
            omega = zeros(numel(t), 3);
            v = t * acceleration.';
            V = [omega, v];

            options = struct( ...
                'knotIntervalSeconds', 0.01, ...
                'sigmaPositionMeters', 1e-3, ...
                'sigmaOrientationRadians', 1e-3, ...
                'sigmaLinearVelocityMps', 1e-3, ...
                'sigmaAngularVelocityRadps', 1e-3, ...
                'jerkSpectralDensityAngular', 1e-4, ...
                'jerkSpectralDensityLinear', 1e-4, ...
                'maxIterations', 10, ...
                'stepTolerance', 1e-8, ...
                'costTolerance', 1e-8);

            smoother = ReplayWnojSmoother(options);
            smoother = smoother.fit(t, R, p, [omega, v]);
            [H, Vd, Ad] = smoother.evaluate(0.10);

            testCase.verifyTrue(smoother.diagnostics.converged);
            testCase.verifyEqual(size(H), [4 4]);
            testCase.verifyEqual(size(Vd), [6 1]);
            testCase.verifyEqual(size(Ad), [6 1]);
            testCase.verifyEqual(H(1:3, 1:3).' * H(1:3, 1:3), ...
                eye(3), 'AbsTol', 1e-10);
            testCase.verifyEqual(H(1:3, 4), p(t == 0.10, :).', ...
                'AbsTol', 5e-3);
            testCase.verifyEqual(Vd(4:6), (0.10 * acceleration), ...
                'AbsTol', 5e-2);
            testCase.verifyEqual(Ad(4:6), acceleration, 'AbsTol', 0.2);
            testCase.verifyEqual(smoother.outputTimes, t, 'AbsTol', 1e-12);
        end

        function testPreservesOriginalOutputGridWhenUsingKnots(testCase)
            t = (0:0.002:0.20).';
            p = [0.2 * t, -0.1 * t, 0.05 * t];
            R = repmat(eye(3), 1, 1, numel(t));
            V = repmat([0.1, -0.05, 0.02, 0.2, -0.1, 0.05], numel(t), 1);

            smoother = ReplayWnojSmoother(struct());
            smoother = smoother.fit(t, R, p, V);

            testCase.verifyEqual(smoother.outputTimes, t, 'AbsTol', 1e-12);
            testCase.verifyLessThan(numel(smoother.t), numel(t));
            [H, Vd, Ad] = smoother.evaluate(t(end));
            testCase.verifyEqual(size(H), [4 4]);
            testCase.verifyEqual(size(Vd), [6 1]);
            testCase.verifyEqual(size(Ad), [6 1]);
        end

        function testConvertsWorldVelocityToBodyAtOutput(testCase)
            t = (0:0.01:0.20).';
            c = cos(pi / 2);
            s = sin(pi / 2);
            Rz = [c -s 0; s c 0; 0 0 1];
            R = repmat(Rz, 1, 1, numel(t));
            p = [t, zeros(numel(t), 2)];
            VWorld = repmat([0 0 0 1 0 0], numel(t), 1);
            VBody = (Rz.' * VWorld(:, 4:6).').';
            VBody = [zeros(numel(t), 3), VBody];

            options = struct( ...
                'sigmaPositionMeters', 1e-4, ...
                'sigmaOrientationRadians', 1e-4, ...
                'sigmaLinearVelocityMps', 1e-4, ...
                'sigmaAngularVelocityRadps', 1e-4, ...
                'jerkSpectralDensityAngular', 1e-4, ...
                'jerkSpectralDensityLinear', 1e-4);
            smoother = ReplayWnojSmoother(options);
            smoother = smoother.fit(t, R, p, VBody);
            [~, Vd, ~] = smoother.evaluate(0.10);

            testCase.verifyEqual(Vd(1:3), [0; 0; 0], 'AbsTol', 1e-3);
            testCase.verifyEqual(Vd(4:6), [0; -1; 0], 'AbsTol', 5e-2);
        end
    end
end
