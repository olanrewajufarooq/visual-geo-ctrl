classdef ReplayWnojSmoother
    %REPLAYWNOJSMOOTHER Nonlinear SE(3) WNOJ batch smoother.
    %   Follows Tang, Yoon, and Barfoot, equations (28)-(42).

    properties (SetAccess = private)
        method
        t
        outputTimes
        H
        V
        A
        options
        diagnostics
    end

    methods
        function obj = ReplayWnojSmoother(options)
            if nargin < 1
                options = struct();
            end
            obj.options = ReplayWnojSmoother.resolveOptions(options);
            obj.method = 'wnoj-se3-batch-v1';
        end

        function obj = fit(obj, t, R, p, VBody)
            options = obj.options;
            [t, HMeas, VBody] = ReplayWnojSmoother.validateMeasurements( ...
                t, R, p, VBody);
            [knotIndices, knotTimes] = ReplayWnojSmoother.selectKnots( ...
                t, options.knotIntervalSeconds);
            HMeas = HMeas(:, :, knotIndices);
            VBody = VBody(knotIndices, :).';
            K = numel(knotTimes);
            [T, V, A] = ReplayWnojSmoother.initializeStates( ...
                knotTimes, HMeas, VBody);
            initialCost = ReplayWnojSmoother.batchCost( ...
                T, V, A, knotTimes, HMeas, VBody, options);
            costHistory = zeros(options.maxIterations + 1, 1);
            costHistory(1) = initialCost;
            stepHistory = zeros(options.maxIterations, 1);
            converged = false;
            for iteration = 1:options.maxIterations
                [residual, jacobian] = ReplayWnojSmoother.linearizeBatch( ...
                    T, V, A, knotTimes, HMeas, VBody, options);
                normalMatrix = jacobian.' * jacobian;
                gradient = jacobian.' * residual;
                damping = options.initialDamping * max(1, max(diag(normalMatrix)));
                delta = -(normalMatrix + damping * eye(size(normalMatrix))) \ gradient;
                stepNorm = norm(delta, inf);
                [TTrial, VTrial, ATrial] = ReplayWnojSmoother.retract(T, V, A, delta);
                trialCost = ReplayWnojSmoother.batchCost( ...
                    TTrial, VTrial, ATrial, knotTimes, HMeas, VBody, options);
                if trialCost > costHistory(iteration)
                    damping = 10 * damping;
                    delta = -(normalMatrix + damping * eye(size(normalMatrix))) \ gradient;
                    [TTrial, VTrial, ATrial] = ReplayWnojSmoother.retract(T, V, A, delta);
                    trialCost = ReplayWnojSmoother.batchCost( ...
                        TTrial, VTrial, ATrial, knotTimes, HMeas, VBody, options);
                end
                T = TTrial;
                V = VTrial;
                A = ATrial;
                costHistory(iteration + 1) = trialCost;
                stepHistory(iteration) = stepNorm;
                if stepNorm < options.stepTolerance || ...
                        abs(costHistory(iteration + 1) - costHistory(iteration)) < options.costTolerance
                    converged = true;
                    break;
                end
            end
            usedIterations = find(costHistory ~= 0, 1, 'last') - 1;
            if isempty(usedIterations)
                usedIterations = 0;
            end
            obj.t = knotTimes(:);
            obj.outputTimes = t(:);
            obj.H = T;
            obj.V = V;
            obj.A = A;
            obj.diagnostics = struct( ...
                'converged', converged, ...
                'iterations', usedIterations, ...
                'initialCost', initialCost, ...
                'finalCost', costHistory(usedIterations + 1), ...
                'costHistory', costHistory(1:usedIterations + 1), ...
                'stepHistory', stepHistory(1:max(usedIterations, 1)), ...
                'knotCount', K);
        end

        function [H, V, A] = evaluate(obj, tq)
            if ~isscalar(tq) || ~isfinite(tq)
                error('fth:ReplayWnoj:InvalidQueryTime', 'Query time must be finite.');
            end
            if isempty(obj.t)
                error('fth:ReplayWnoj:NotFitted', ...
                    'The smoother must be fitted before evaluation.');
            end
            tq = min(max(tq, obj.t(1)), obj.t(end));
            if tq == obj.t(end)
                i = numel(obj.t) - 1;
            else
                i = min(find(obj.t <= tq, 1, 'last'), numel(obj.t) - 1);
            end
            tau = tq - obj.t(i);
            [H, V, A] = ReplayWnojSmoother.interpolate( ...
                obj.H(:, :, i), obj.V(:, i), obj.A(:, i), ...
                obj.H(:, :, i + 1), obj.V(:, i + 1), obj.A(:, i + 1), ...
                obj.t(i + 1) - obj.t(i), tau, obj.options.jerkSpectralDensity);
        end

    end

    methods (Static)
        function [Phi, Q] = transition(dt, Qc)
            I = eye(6);
            Phi = [I, dt * I, 0.5 * dt^2 * I; ...
                zeros(6), I, dt * I; zeros(6), zeros(6), I];
            Q = [dt^5 / 20 * Qc, dt^4 / 8 * Qc, dt^3 / 6 * Qc; ...
                dt^4 / 8 * Qc, dt^3 / 3 * Qc, dt^2 / 2 * Qc; ...
                dt^3 / 6 * Qc, dt^2 / 2 * Qc, dt * Qc];
        end

        function e = priorResidual(Ti, Vi, Ai, Tj, Vj, Aj, dt)
            eta = fth.se3.logSE3(Tj / Ti);
            Jinv = ReplayWnojSmoother.leftJacobianInverse(eta);
            localVj = Jinv * Vj;
            localAj = -0.5 * fth.se3.adV(localVj) * Vj + Jinv * Aj;
            gammaI = [zeros(6, 1); Vi; Ai];
            gammaJ = [eta; localVj; localAj];
            [Phi, ~] = ReplayWnojSmoother.transition(dt, eye(6));
            e = gammaJ - Phi * gammaI;
        end

    end

    methods

        function diagnostics = validate(obj)
            times = obj.outputTimes(:);
            poseResidual = zeros(numel(times), 6);
            twistResidual = zeros(numel(times), 6);
            accelResidual = zeros(numel(times), 6);
            for k = 2:numel(times) - 1
                dt = times(k + 1) - times(k - 1);
                [H0, V0, A0] = obj.evaluate(times(k - 1));
                [H1, V1, A1] = obj.evaluate(times(k + 1));
                poseResidual(k, :) = fth.se3.logSE3(H1 / H0).' - dt * V0.';
                twistResidual(k, :) = ((V1 - V0) / dt - A0).';
                accelResidual(k, :) = ((A1 - A0) / dt).';
            end
            diagnostics = struct( ...
                'maxPoseTwistResidual', max(abs(poseResidual), [], 'all'), ...
                'maxTwistAccelerationResidual', max(abs(twistResidual), [], 'all'), ...
                'maxJerkResidual', max(abs(accelResidual), [], 'all'));
        end
    end

    methods (Static, Access = private)
        function options = resolveOptions(options)
            if nargin < 1 || isempty(options)
                options = struct();
            end
            defaults = struct( ...
                'knotIntervalSeconds', 0.01, ...
                'poseSigma', 0.01, ...
                'twistSigma', 0.1, ...
                'jerkSpectralDensity', eye(6), ...
                'maxIterations', 15, ...
                'stepTolerance', 1e-8, ...
                'costTolerance', 1e-8, ...
                'initialDamping', 1e-6);
            names = fieldnames(defaults);
            for k = 1:numel(names)
                if ~isfield(options, names{k}) || isempty(options.(names{k}))
                    options.(names{k}) = defaults.(names{k});
                end
            end
            if isscalar(options.jerkSpectralDensity)
                options.jerkSpectralDensity = options.jerkSpectralDensity * eye(6);
            end
            if ~isequal(size(options.jerkSpectralDensity), [6 6]) || ...
                    min(eig(options.jerkSpectralDensity)) <= 0
                error('fth:ReplayWnoj:InvalidOptions', ...
                    'jerkSpectralDensity must be positive definite 6x6.');
            end
        end

        function [t, H, V] = validateMeasurements(t, R, p, V)
            t = t(:);
            n = numel(t);
            if size(R, 1) ~= 3 || size(R, 2) ~= 3 || size(R, 3) ~= n || ...
                    size(p, 1) ~= n || size(V, 1) ~= n || size(V, 2) ~= 6
                error('fth:ReplayWnoj:InvalidMeasurements', 'Inputs are not aligned.');
            end
            if n < 3 || any(~isfinite(t)) || any(diff(t) <= 0) || ...
                    any(~isfinite(p(:))) || any(~isfinite(V(:)))
                error('fth:ReplayWnoj:InvalidMeasurements', 'Invalid measurements.');
            end
            H = zeros(4, 4, n);
            for k = 1:n
                Rk = R(:, :, k);
                if norm(Rk.' * Rk - eye(3), 'fro') > 1e-5 || det(Rk) <= 0
                    error('fth:ReplayWnoj:InvalidRotations', 'Rotation %d is invalid.', k);
                end
                H(:, :, k) = [Rk, p(k, :).'; 0 0 0 1];
            end
        end

        function [indices, knots] = selectKnots(t, interval)
            indices = 1;
            last = 1;
            for k = 2:numel(t)
                if t(k) - t(last) >= interval
                    indices(end + 1) = k; %#ok<AGROW>
                    last = k;
                end
            end
            if indices(end) ~= numel(t)
                indices(end + 1) = numel(t);
            end
            if numel(indices) < 3
                indices = unique(round(linspace(1, numel(t), 3)));
            end
            knots = t(indices);
        end

        function [T, V, A] = initializeStates(t, HMeas, VMeas)
            K = numel(t);
            T = HMeas;
            V = VMeas;
            A = zeros(6, K);
            for k = 2:K - 1
                A(:, k) = (V(:, k + 1) - V(:, k - 1)) / (t(k + 1) - t(k - 1));
            end
            A(:, 1) = A(:, 2);
            A(:, K) = A(:, K - 1);
        end

        function cost = batchCost(T, V, A, t, HMeas, VMeas, options)
            residual = ReplayWnojSmoother.residualVector( ...
                T, V, A, t, HMeas, VMeas, options);
            cost = 0.5 * (residual.' * residual);
        end

        function [r, J] = linearizeBatch(T, V, A, t, HMeas, VMeas, options)
            K = numel(t);
            rows = 12 * K + 18 * (K - 1);
            cols = 18 * K;
            r = zeros(rows, 1);
            J = spalloc(rows, cols, 12 * 18 * K + 18 * 36 * (K - 1));
            epsilon = 1e-6;
            row = 0;
            for k = 1:K
                base = ReplayWnojSmoother.measurementResidual( ...
                    T(:, :, k), V(:, k), HMeas(:, :, k), VMeas(:, k), options);
                r(row + (1:12)) = base;
                for local = 1:18
                    [Tp, Vp, Ap] = ReplayWnojSmoother.perturbState( ...
                        T, V, A, k, local, epsilon);
                    perturbed = ReplayWnojSmoother.measurementResidual( ...
                        Tp(:, :, k), Vp(:, k), HMeas(:, :, k), ...
                        VMeas(:, k), options);
                    J(row + (1:12), 18 * (k - 1) + local) = ...
                        (perturbed - base) / epsilon;
                end
                row = row + 12;
            end
            for k = 1:K - 1
                dt = t(k + 1) - t(k);
                base = ReplayWnojSmoother.priorWhitenedResidual( ...
                    T(:, :, k), V(:, k), A(:, k), T(:, :, k + 1), ...
                    V(:, k + 1), A(:, k + 1), dt, options.jerkSpectralDensity);
                r(row + (1:18)) = base;
                for local = 1:36
                    if local <= 18
                        knot = k;
                        stateLocal = local;
                    else
                        knot = k + 1;
                        stateLocal = local - 18;
                    end
                    [Tp, Vp, Ap] = ReplayWnojSmoother.perturbState( ...
                        T, V, A, knot, stateLocal, epsilon);
                    perturbed = ReplayWnojSmoother.priorWhitenedResidual( ...
                        Tp(:, :, k), Vp(:, k), Ap(:, k), Tp(:, :, k + 1), ...
                        Vp(:, k + 1), Ap(:, k + 1), dt, options.jerkSpectralDensity);
                    J(row + (1:18), 18 * (k - 1) + local) = ...
                        (perturbed - base) / epsilon;
                end
                row = row + 18;
            end
        end

        function r = measurementResidual(T, V, HMeas, VMeas, options)
            r = [fth.se3.logSE3(HMeas / T) / options.poseSigma; ...
                (V - VMeas) / options.twistSigma];
        end

        function r = priorWhitenedResidual(Ti, Vi, Ai, Tj, Vj, Aj, dt, Qc)
            e = ReplayWnojSmoother.priorResidual(Ti, Vi, Ai, Tj, Vj, Aj, dt);
            [~, Q] = ReplayWnojSmoother.transition(dt, Qc);
            r = chol(Q, 'lower') \ e;
        end

        function [T, V, A] = perturbState(T, V, A, knot, local, epsilon)
            delta = zeros(18, 1);
            delta(local) = epsilon;
            offset = 18 * (knot - 1);
            T(:, :, knot) = fth.se3.expSE3(fth.se3.vec2tilde( ...
                delta(1:6))) * T(:, :, knot);
            V(:, knot) = V(:, knot) + delta(7:12);
            A(:, knot) = A(:, knot) + delta(13:18);
        end

        function r = residualVector(T, V, A, t, HMeas, VMeas, options)
            K = numel(t);
            r = zeros(12 * K + 18 * (K - 1), 1);
            cursor = 0;
            for k = 1:K
                r(cursor + (1:6)) = fth.se3.logSE3(HMeas(:, :, k) / T(:, :, k)) / options.poseSigma;
                cursor = cursor + 6;
                r(cursor + (1:6)) = (V(:, k) - VMeas(:, k)) / options.twistSigma;
                cursor = cursor + 6;
            end
            for k = 1:K - 1
                dt = t(k + 1) - t(k);
                e = ReplayWnojSmoother.priorResidual( ...
                    T(:, :, k), V(:, k), A(:, k), T(:, :, k + 1), ...
                    V(:, k + 1), A(:, k + 1), dt);
                [~, Q] = ReplayWnojSmoother.transition(dt, options.jerkSpectralDensity);
                r(cursor + (1:18)) = chol(Q, 'lower') \ e;
                cursor = cursor + 18;
            end
            r = r(1:cursor);
        end

        function [T, V, A] = retract(T, V, A, delta)
            K = size(V, 2);
            for k = 1:K
                offset = 18 * (k - 1);
                T(:, :, k) = fth.se3.expSE3(fth.se3.vec2tilde( ...
                    delta(offset + (1:6)))) * T(:, :, k);
                V(:, k) = V(:, k) + delta(offset + (7:12));
                A(:, k) = A(:, k) + delta(offset + (13:18));
            end
        end

        function [H, V, A] = interpolate(Ti, Vi, Ai, Tj, Vj, Aj, dt, tau, Qc)
            [PhiTau, QTau] = ReplayWnojSmoother.transition(tau, Qc);
            [PhiDt, QDt] = ReplayWnojSmoother.transition(dt, Qc);
            if tau <= eps
                Omega = zeros(18);
            else
                [PhiRemain, ~] = ReplayWnojSmoother.transition(dt - tau, Qc);
                Omega = QTau * PhiRemain.' / QDt;
            end
            Lambda = PhiTau - Omega * PhiDt;
            eta = fth.se3.logSE3(Tj / Ti);
            Jinv = ReplayWnojSmoother.leftJacobianInverse(eta);
            localVj = Jinv * Vj;
            localAj = -0.5 * fth.se3.adV(localVj) * Vj + Jinv * Aj;
            gamma = Lambda * [zeros(6, 1); Vi; Ai] + Omega * [eta; localVj; localAj];
            xi = gamma(1:6);
            localV = gamma(7:12);
            localA = gamma(13:18);
            J = inv(ReplayWnojSmoother.leftJacobianInverse(xi));
            V = J * localV;
            A = J * (localA + 0.5 * fth.se3.adV(localV) * V);
            H = fth.se3.expSE3(fth.se3.vec2tilde(xi)) * Ti;
        end

        function Jinv = leftJacobianInverse(xi)
            ad = fth.se3.adV(xi);
            Jinv = eye(6) - 0.5 * ad + (1 / 12) * ad^2 - ...
                (1 / 720) * ad^4 + (1 / 30240) * ad^6;
        end
    end
end
