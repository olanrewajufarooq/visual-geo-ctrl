classdef ReplayWnojSmoother
    %REPLAYWNOJSMOOTHER Sparse nonlinear SE(3) WNOJ batch smoother.
    %   The public API uses the project convention Hdot = H*hat(V), where
    %   H maps body coordinates to world coordinates. Internally the class
    %   uses the convention of Tang, Yoon, and Barfoot:
    %       T = inv(H), varpi = -V, dotvarpi = -A,
    %       Tdot = hat(varpi)*T.

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

    properties (Access = private)
        paperT
        paperVelocity
        paperAcceleration
    end

    methods
        function obj = ReplayWnojSmoother(options)
            if nargin < 1
                options = struct();
            end
            obj.options = obj.resolveOptions(options);
            obj.method = 'wnoj';
        end

        function obj = fit(obj, t, R, p, VBody)
            [measurementTimes, HAll, VBody] = obj.validateMeasurements( ...
                t, R, p, VBody);
            [knotIndices, knotTimes] = obj.selectKnots( ...
                measurementTimes, obj.options.knotIntervalSeconds);

            HMeas = HAll(:, :, knotIndices);
            VProjectMeas = VBody(knotIndices, :).';
            [TMeas, WMeas] = obj.toPaperMeasurements( ...
                HMeas, VProjectMeas);
            [T, W, D] = obj.initializeStates(knotTimes, TMeas, WMeas);

            currentCost = obj.batchCost( ...
                T, W, D, knotTimes, TMeas, WMeas);
            costHistory = currentCost;
            stepHistory = zeros(0, 1);
            dampingHistory = zeros(0, 1);
            converged = false;
            terminationReason = 'maximum iterations';
            normalNnz = 0;
            stateDimension = 18 * numel(knotTimes);
            damping = obj.options.initialDamping;
            gradientInfinityNorm = NaN;

            for iteration = 1:obj.options.maxIterations
                [residual, jacobian] = obj.linearizeBatch( ...
                    T, W, D, knotTimes, TMeas, WMeas);
                normalMatrix = jacobian.' * jacobian;
                gradient = jacobian.' * residual;
                gradientInfinityNorm = norm(gradient, inf);
                normalNnz = nnz(normalMatrix);

                if gradientInfinityNorm <= obj.options.gradientTolerance
                    converged = true;
                    terminationReason = 'gradient tolerance';
                    break;
                end

                diagonalScale = max(full(diag(normalMatrix)), 1);
                accepted = false;
                acceptedStep = [];
                acceptedCost = currentCost;
                acceptedDamping = damping;

                for trial = 1:obj.options.maxDampingTrials
                    dampingMatrix = spdiags( ...
                        damping * diagonalScale, 0, stateDimension, stateDimension);
                    delta = -(normalMatrix + dampingMatrix) \ gradient;
                    if any(~isfinite(delta))
                        damping = 10 * damping;
                        continue;
                    end

                    [TTrial, WTrial, DTrial] = obj.retract(T, W, D, delta);
                    trialCost = obj.batchCost( ...
                        TTrial, WTrial, DTrial, knotTimes, TMeas, WMeas);
                    if isfinite(trialCost) && trialCost < currentCost
                        accepted = true;
                        acceptedStep = delta;
                        acceptedCost = trialCost;
                        acceptedDamping = damping;
                        T = TTrial;
                        W = WTrial;
                        D = DTrial;
                        break;
                    end
                    damping = 10 * damping;
                end

                if ~accepted
                    terminationReason = 'no decreasing LM step';
                    break;
                end

                previousCost = currentCost;
                currentCost = acceptedCost;
                costHistory(end + 1, 1) = currentCost; %#ok<AGROW>
                stepHistory(end + 1, 1) = norm(acceptedStep, inf); %#ok<AGROW>
                dampingHistory(end + 1, 1) = acceptedDamping; %#ok<AGROW>
                damping = max(obj.options.minimumDamping, acceptedDamping / 3);

                if stepHistory(end) <= obj.options.stepTolerance
                    converged = true;
                    terminationReason = 'step tolerance';
                    break;
                end
                relativeDecrease = (previousCost - currentCost) / max(1, previousCost);
                if relativeDecrease <= obj.options.costTolerance
                    converged = true;
                    terminationReason = 'cost tolerance';
                    break;
                end
            end

            if ~converged && obj.options.requireConvergence
                error('fth:ReplayWnoj:NoConvergence', ...
                    'WNOJ optimization stopped: %s.', terminationReason);
            end

            obj.t = knotTimes(:);
            obj.outputTimes = measurementTimes(:);
            obj.paperT = T;
            obj.paperVelocity = W;
            obj.paperAcceleration = D;
            [obj.H, obj.V, obj.A] = obj.paperArraysToProject(T, W, D);
            obj.diagnostics = struct( ...
                'converged', converged, ...
                'terminationReason', terminationReason, ...
                'iterations', numel(stepHistory), ...
                'initialCost', costHistory(1), ...
                'finalCost', currentCost, ...
                'costHistory', costHistory, ...
                'stepHistory', stepHistory, ...
                'dampingHistory', dampingHistory, ...
                'finalStepInfinityNorm', obj.lastOrNaN(stepHistory), ...
                'finalRelativeCostDecrease', ...
                obj.finalRelativeDecrease(costHistory), ...
                'linearizationGradientInfinityNorm', gradientInfinityNorm, ...
                'knotCount', numel(knotTimes), ...
                'stateDimension', stateDimension, ...
                'normalNnz', normalNnz);
            if ~converged && obj.options.warnOnNonConvergence
                warning('fth:ReplayWnoj:NoConvergence', ...
                    ['WNOJ optimization stopped after %d iterations (%s). ' ...
                    'Final relative cost decrease %.3g; final step %.3g.'], ...
                    obj.diagnostics.iterations, terminationReason, ...
                    obj.diagnostics.finalRelativeCostDecrease, ...
                    obj.diagnostics.finalStepInfinityNorm);
            end
        end

        function [H, V, A] = evaluate(obj, tq)
            if ~isscalar(tq) || ~isfinite(tq)
                error('fth:ReplayWnoj:InvalidQueryTime', ...
                    'Query time must be a finite scalar.');
            end
            if isempty(obj.t)
                error('fth:ReplayWnoj:NotFitted', ...
                    'The smoother must be fitted before evaluation.');
            end

            tq = min(max(tq, obj.t(1)), obj.t(end));
            if tq <= obj.t(1)
                [H, V, A] = obj.paperStateToProject( ...
                    obj.paperT(:, :, 1), obj.paperVelocity(:, 1), ...
                    obj.paperAcceleration(:, 1));
                return;
            end
            if tq >= obj.t(end)
                last = numel(obj.t);
                [H, V, A] = obj.paperStateToProject( ...
                    obj.paperT(:, :, last), obj.paperVelocity(:, last), ...
                    obj.paperAcceleration(:, last));
                return;
            end

            i = find(obj.t <= tq, 1, 'last');
            dt = obj.t(i + 1) - obj.t(i);
            tau = tq - obj.t(i);
            [T, W, D] = obj.interpolate( ...
                obj.paperT(:, :, i), obj.paperVelocity(:, i), ...
                obj.paperAcceleration(:, i), obj.paperT(:, :, i + 1), ...
                obj.paperVelocity(:, i + 1), ...
                obj.paperAcceleration(:, i + 1), dt, tau);
            [H, V, A] = obj.paperStateToProject(T, W, D);
        end

        function [Phi, Q] = transition(obj, dt, Qc) %#ok<INUSL>
            I = eye(6);
            Phi = [I, dt * I, 0.5 * dt^2 * I; ...
                zeros(6), I, dt * I; zeros(6), zeros(6), I];
            Q = [dt^5 / 20 * Qc, dt^4 / 8 * Qc, dt^3 / 6 * Qc; ...
                dt^4 / 8 * Qc, dt^3 / 3 * Qc, dt^2 / 2 * Qc; ...
                dt^3 / 6 * Qc, dt^2 / 2 * Qc, dt * Qc];
        end

        function residual = priorResidual(obj, Hi, Vi, Ai, Hj, Vj, Aj, dt)
            [Ti, Wi, Di] = obj.projectStateToPaper(Hi, Vi, Ai);
            [Tj, Wj, Dj] = obj.projectStateToPaper(Hj, Vj, Aj);
            residual = obj.paperPriorResidual( ...
                Ti, Wi, Di, Tj, Wj, Dj, dt);
        end

        function diagnostics = validate(obj)
            if isempty(obj.outputTimes)
                error('fth:ReplayWnoj:NotFitted', ...
                    'The smoother must be fitted before validation.');
            end
            times = obj.outputTimes(:);
            poseTwistResidual = zeros(max(numel(times) - 1, 0), 6);
            twistAccelerationResidual = zeros(max(numel(times) - 1, 0), 6);
            for k = 1:numel(times) - 1
                dt = times(k + 1) - times(k);
                [H0, V0, A0] = obj.evaluate(times(k));
                [H1, V1, ~] = obj.evaluate(times(k + 1));
                poseTwistResidual(k, :) = ...
                    (obj.logPose(H0 \ H1) / dt - V0).';
                twistAccelerationResidual(k, :) = ...
                    ((V1 - V0) / dt - A0).';
            end
            diagnostics = struct( ...
                'maxPoseTwistResidual', ...
                max(abs(poseTwistResidual), [], 'all'), ...
                'maxTwistAccelerationResidual', ...
                max(abs(twistAccelerationResidual), [], 'all'));
        end
    end

    methods (Access = private)
        function options = resolveOptions(obj, supplied) %#ok<INUSL>
            if isempty(supplied)
                supplied = struct();
            end
            if ~isstruct(supplied) || ~isscalar(supplied)
                error('fth:ReplayWnoj:InvalidOptions', ...
                    'Options must be a scalar struct.');
            end

            defaults = struct( ...
                'knotIntervalSeconds', 0.01, ...
                'sigmaPositionMeters', 0.01, ...
                'sigmaOrientationRadians', 0.01, ...
                'sigmaLinearVelocityMps', 0.10, ...
                'sigmaAngularVelocityRadps', 0.10, ...
                'jerkSpectralDensityAngular', 1.0, ...
                'jerkSpectralDensityLinear', 1.0, ...
                'maxIterations', 15, ...
                'maxDampingTrials', 8, ...
                'stepTolerance', 1e-7, ...
                'gradientTolerance', 1e-6, ...
                'costTolerance', 1e-7, ...
                'initialDamping', 1e-6, ...
                'minimumDamping', 1e-12, ...
                'finiteDifferenceStep', 1e-6, ...
                'requireConvergence', false, ...
                'warnOnNonConvergence', true);

            if isfield(supplied, 'poseSigma')
                if ~isfield(supplied, 'sigmaPositionMeters')
                    supplied.sigmaPositionMeters = supplied.poseSigma;
                end
                if ~isfield(supplied, 'sigmaOrientationRadians')
                    supplied.sigmaOrientationRadians = supplied.poseSigma;
                end
            end
            if isfield(supplied, 'twistSigma')
                if ~isfield(supplied, 'sigmaLinearVelocityMps')
                    supplied.sigmaLinearVelocityMps = supplied.twistSigma;
                end
                if ~isfield(supplied, 'sigmaAngularVelocityRadps')
                    supplied.sigmaAngularVelocityRadps = supplied.twistSigma;
                end
            end

            names = fieldnames(defaults);
            options = supplied;
            for k = 1:numel(names)
                if ~isfield(options, names{k}) || isempty(options.(names{k}))
                    options.(names{k}) = defaults.(names{k});
                end
            end

            positiveScalars = {'knotIntervalSeconds', ...
                'sigmaPositionMeters', 'sigmaOrientationRadians', ...
                'sigmaLinearVelocityMps', 'sigmaAngularVelocityRadps', ...
                'maxIterations', 'maxDampingTrials', 'stepTolerance', ...
                'gradientTolerance', 'costTolerance', 'initialDamping', ...
                'minimumDamping', 'finiteDifferenceStep'};
            for k = 1:numel(positiveScalars)
                value = options.(positiveScalars{k});
                if ~isscalar(value) || ~isfinite(value) || value <= 0
                    error('fth:ReplayWnoj:InvalidOptions', ...
                        '%s must be a positive finite scalar.', positiveScalars{k});
                end
            end
            integerScalars = {'maxIterations', 'maxDampingTrials'};
            for k = 1:numel(integerScalars)
                value = options.(integerScalars{k});
                if value ~= round(value)
                    error('fth:ReplayWnoj:InvalidOptions', ...
                        '%s must be a positive integer.', integerScalars{k});
                end
            end
            logicalScalars = {'requireConvergence', 'warnOnNonConvergence'};
            for k = 1:numel(logicalScalars)
                value = options.(logicalScalars{k});
                if ~isscalar(value) || ...
                        ~(islogical(value) || ...
                        (isnumeric(value) && isfinite(value) && ismember(value, [0 1])))
                    error('fth:ReplayWnoj:InvalidOptions', ...
                        '%s must be a logical scalar.', logicalScalars{k});
                end
                options.(logicalScalars{k}) = logical(value);
            end
            if isfield(supplied, 'jerkSpectralDensity') && ...
                    ~isempty(supplied.jerkSpectralDensity)
                Qc = supplied.jerkSpectralDensity;
                if isscalar(Qc)
                    Qc = Qc * eye(6);
                end
            else
                angular = obj.expandAxisVariance( ...
                    options.jerkSpectralDensityAngular, ...
                    'jerkSpectralDensityAngular');
                linear = obj.expandAxisVariance( ...
                    options.jerkSpectralDensityLinear, ...
                    'jerkSpectralDensityLinear');
                Qc = diag([angular; linear]);
            end
            if ~isequal(size(Qc), [6 6]) || any(~isfinite(Qc(:))) || ...
                    norm(Qc - Qc.', 'fro') > 1e-10 || ...
                    min(eig((Qc + Qc.') / 2)) <= 0
                error('fth:ReplayWnoj:InvalidOptions', ...
                    'The jerk spectral density must be positive-definite 6x6.');
            end
            options.jerkSpectralDensity = Qc;
            options.poseStd = [repmat(options.sigmaOrientationRadians, 3, 1); ...
                repmat(options.sigmaPositionMeters, 3, 1)];
            options.twistStd = [repmat(options.sigmaAngularVelocityRadps, 3, 1); ...
                repmat(options.sigmaLinearVelocityMps, 3, 1)];
        end

        function values = expandAxisVariance(obj, value, name) %#ok<INUSL>
            if isscalar(value)
                values = repmat(value, 3, 1);
            else
                values = value(:);
            end
            if numel(values) ~= 3 || any(~isfinite(values)) || any(values <= 0)
                error('fth:ReplayWnoj:InvalidOptions', ...
                    '%s must be a positive scalar or three-vector.', name);
            end
        end

        function [t, H, V] = validateMeasurements(obj, t, R, p, V) %#ok<INUSL>
            t = t(:);
            n = numel(t);
            if n < 3 || size(R, 1) ~= 3 || size(R, 2) ~= 3 || ...
                    size(R, 3) ~= n || ~isequal(size(p), [n 3]) || ...
                    ~isequal(size(V), [n 6])
                error('fth:ReplayWnoj:InvalidMeasurements', ...
                    'Pose, twist, and time inputs are not aligned.');
            end
            if any(~isfinite(t)) || any(diff(t) <= 0) || ...
                    any(~isfinite(p(:))) || any(~isfinite(V(:)))
                error('fth:ReplayWnoj:InvalidMeasurements', ...
                    'Measurements must be finite with strictly increasing time.');
            end
            H = zeros(4, 4, n);
            for k = 1:n
                Rk = R(:, :, k);
                if any(~isfinite(Rk(:))) || ...
                        norm(Rk.' * Rk - eye(3), 'fro') > 1e-5 || ...
                        abs(det(Rk) - 1) > 1e-5
                    error('fth:ReplayWnoj:InvalidRotations', ...
                        'Rotation measurement %d is invalid.', k);
                end
                H(:, :, k) = [Rk, p(k, :).'; 0 0 0 1];
            end
        end

        function [indices, knots] = selectKnots(obj, t, interval) %#ok<INUSL>
            indices = zeros(numel(t), 1);
            count = 1;
            indices(count) = 1;
            lastIndex = 1;
            for k = 2:numel(t) - 1
                if t(k) - t(lastIndex) >= interval
                    count = count + 1;
                    indices(count) = k;
                    lastIndex = k;
                end
            end
            if indices(count) ~= numel(t)
                count = count + 1;
                indices(count) = numel(t);
            end
            indices = indices(1:count);
            if numel(indices) < 3
                indices = unique(round(linspace(1, numel(t), 3))).';
            end
            knots = t(indices);
        end

        function [T, W] = toPaperMeasurements(obj, H, VProject)
            K = size(H, 3);
            T = zeros(4, 4, K);
            for k = 1:K
                T(:, :, k) = obj.inversePose(H(:, :, k));
            end
            W = -VProject;
        end

        function [T, W, D] = initializeStates(obj, t, TMeas, WMeas) %#ok<INUSL>
            K = numel(t);
            T = TMeas;
            W = WMeas;
            D = zeros(6, K);
            for k = 2:K - 1
                D(:, k) = (W(:, k + 1) - W(:, k - 1)) / ...
                    (t(k + 1) - t(k - 1));
            end
            D(:, 1) = (W(:, 2) - W(:, 1)) / (t(2) - t(1));
            D(:, K) = (W(:, K) - W(:, K - 1)) / (t(K) - t(K - 1));
        end

        function cost = batchCost(obj, T, W, D, t, TMeas, WMeas)
            residual = obj.residualVector(T, W, D, t, TMeas, WMeas);
            cost = 0.5 * full(residual.' * residual);
        end

        function residual = residualVector(obj, T, W, D, t, TMeas, WMeas)
            K = numel(t);
            residual = zeros(12 * K + 18 * (K - 1), 1);
            row = 0;
            for k = 1:K
                rows = row + (1:12);
                residual(rows) = obj.measurementResidual( ...
                    T(:, :, k), W(:, k), TMeas(:, :, k), WMeas(:, k));
                row = row + 12;
            end
            for k = 1:K - 1
                rows = row + (1:18);
                residual(rows) = obj.priorWhitenedResidual( ...
                    T(:, :, k), W(:, k), D(:, k), T(:, :, k + 1), ...
                    W(:, k + 1), D(:, k + 1), t(k + 1) - t(k));
                row = row + 18;
            end
        end

        function [residual, jacobian] = linearizeBatch( ...
                obj, T, W, D, t, TMeas, WMeas)
            K = numel(t);
            rowCount = 12 * K + 18 * (K - 1);
            columnCount = 18 * K;
            residual = zeros(rowCount, 1);
            jacobian = spalloc(rowCount, columnCount, 720 * K);
            row = 0;

            for k = 1:K
                rows = row + (1:12);
                columns = 18 * (k - 1) + (1:18);
                [factorResidual, factorJacobian] = obj.linearizeMeasurement( ...
                    T(:, :, k), W(:, k), TMeas(:, :, k), WMeas(:, k));
                residual(rows) = factorResidual;
                jacobian(rows, columns) = sparse(factorJacobian);
                row = row + 12;
            end

            for k = 1:K - 1
                rows = row + (1:18);
                columns = 18 * (k - 1) + (1:36);
                [factorResidual, factorJacobian] = obj.linearizePrior( ...
                    T(:, :, k), W(:, k), D(:, k), T(:, :, k + 1), ...
                    W(:, k + 1), D(:, k + 1), t(k + 1) - t(k));
                residual(rows) = factorResidual;
                jacobian(rows, columns) = sparse(factorJacobian);
                row = row + 18;
            end
        end

        function [residual, jacobian] = linearizeMeasurement( ...
                obj, T, W, TMeas, WMeas)
            residual = obj.measurementResidual(T, W, TMeas, WMeas);
            jacobian = zeros(12, 18);
            epsilon = obj.options.finiteDifferenceStep;
            for column = 1:6
                perturbation = zeros(6, 1);
                perturbation(column) = epsilon;
                TPlus = obj.expPose(perturbation) * T;
                plus = obj.measurementResidual(TPlus, W, TMeas, WMeas);
                jacobian(:, column) = (plus - residual) / epsilon;
            end
            jacobian(7:12, 7:12) = diag(1 ./ obj.options.twistStd);
        end

        function residual = measurementResidual(obj, T, W, TMeas, WMeas)
            residual = [obj.logPose(TMeas / T) ./ obj.options.poseStd; ...
                (W - WMeas) ./ obj.options.twistStd];
        end

        function [residual, jacobian] = linearizePrior( ...
                obj, Ti, Wi, Di, Tj, Wj, Dj, dt)
            rawResidual = obj.paperPriorResidual(Ti, Wi, Di, Tj, Wj, Dj, dt);
            rawJacobian = zeros(18, 36);
            epsilon = obj.options.finiteDifferenceStep;

            for column = 1:6
                perturbation = zeros(6, 1);
                perturbation(column) = epsilon;
                plus = obj.paperPriorResidual( ...
                    obj.expPose(perturbation) * Ti, Wi, Di, Tj, Wj, Dj, dt);
                rawJacobian(:, column) = (plus - rawResidual) / epsilon;

                plus = obj.paperPriorResidual( ...
                    Ti, Wi, Di, obj.expPose(perturbation) * Tj, Wj, Dj, dt);
                rawJacobian(:, 18 + column) = (plus - rawResidual) / epsilon;

                WjPlus = Wj;
                WjPlus(column) = WjPlus(column) + epsilon;
                plus = obj.paperPriorResidual( ...
                    Ti, Wi, Di, Tj, WjPlus, Dj, dt);
                rawJacobian(:, 24 + column) = ...
                    (plus - rawResidual) / epsilon;
            end

            eta = obj.logPose(Tj / Ti);
            [Jeta, ~] = obj.leftJacobianAndDerivative(eta, zeros(6, 1));
            Jinv = Jeta \ eye(6);
            I = eye(6);
            rawJacobian(1:6, 7:12) = -dt * I;
            rawJacobian(1:6, 13:18) = -0.5 * dt^2 * I;
            rawJacobian(7:12, 7:12) = -I;
            rawJacobian(7:12, 13:18) = -dt * I;
            rawJacobian(13:18, 13:18) = -I;
            rawJacobian(13:18, 31:36) = Jinv;

            [~, Q] = obj.transition(dt, obj.options.jerkSpectralDensity);
            lower = chol(Q, 'lower');
            residual = lower \ rawResidual;
            jacobian = lower \ rawJacobian;
        end

        function residual = priorWhitenedResidual( ...
                obj, Ti, Wi, Di, Tj, Wj, Dj, dt)
            rawResidual = obj.paperPriorResidual(Ti, Wi, Di, Tj, Wj, Dj, dt);
            [~, Q] = obj.transition(dt, obj.options.jerkSpectralDensity);
            residual = chol(Q, 'lower') \ rawResidual;
        end

        function residual = paperPriorResidual( ...
                obj, Ti, Wi, Di, Tj, Wj, Dj, dt)
            eta = obj.logPose(Tj / Ti);
            [localWj, localDj] = obj.localKinematics(eta, Wj, Dj);
            gammaI = [zeros(6, 1); Wi; Di];
            gammaJ = [eta; localWj; localDj];
            [Phi, ~] = obj.transition(dt, eye(6));
            residual = gammaJ - Phi * gammaI;
        end

        function [T, W, D] = retract(obj, T, W, D, delta)
            K = size(W, 2);
            for k = 1:K
                offset = 18 * (k - 1);
                T(:, :, k) = obj.expPose(delta(offset + (1:6))) * T(:, :, k);
                W(:, k) = W(:, k) + delta(offset + (7:12));
                D(:, k) = D(:, k) + delta(offset + (13:18));
            end
        end

        function [T, W, D] = interpolate( ...
                obj, Ti, Wi, Di, Tj, Wj, Dj, dt, tau)
            Qc = obj.options.jerkSpectralDensity;
            [PhiTau, QTau] = obj.transition(tau, Qc);
            [PhiDt, QDt] = obj.transition(dt, Qc);
            [PhiRemaining, ~] = obj.transition(dt - tau, Qc);
            Omega = QTau * PhiRemaining.' / QDt;
            Lambda = PhiTau - Omega * PhiDt;

            eta = obj.logPose(Tj / Ti);
            [localWj, localDj] = obj.localKinematics(eta, Wj, Dj);
            gamma = Lambda * [zeros(6, 1); Wi; Di] + ...
                Omega * [eta; localWj; localDj];

            xi = gamma(1:6);
            xiDot = gamma(7:12);
            xiDDot = gamma(13:18);
            [Jxi, JxiDot] = obj.leftJacobianAndDerivative(xi, xiDot);
            W = Jxi * xiDot;
            D = Jxi * xiDDot + JxiDot * xiDot;
            T = obj.expPose(xi) * Ti;
        end

        function [T, W, D] = projectStateToPaper(obj, H, V, A)
            T = obj.inversePose(H);
            W = -V;
            D = -A;
        end

        function [H, V, A] = paperStateToProject(obj, T, W, D)
            H = obj.inversePose(T);
            V = -W;
            A = -D;
        end

        function [H, V, A] = paperArraysToProject(obj, T, W, D)
            K = size(T, 3);
            H = zeros(4, 4, K);
            for k = 1:K
                H(:, :, k) = obj.inversePose(T(:, :, k));
            end
            V = -W;
            A = -D;
        end

        function inverse = inversePose(obj, pose) %#ok<INUSL>
            R = pose(1:3, 1:3);
            p = pose(1:3, 4);
            inverse = [R.', -R.' * p; 0 0 0 1];
        end

        function pose = expPose(obj, vector) %#ok<INUSL>
            pose = fth.se3.expSE3(fth.se3.vec2tilde(vector));
        end

        function vector = logPose(obj, pose) %#ok<INUSL>
            vector = fth.se3.logSE3(pose);
        end

        function matrix = adjointMatrix(obj, vector) %#ok<INUSL>
            matrix = fth.se3.adV(vector);
        end

        function Jinv = leftJacobianInverse(obj, xi)
            ad = obj.adjointMatrix(xi);
            ad2 = ad * ad;
            Jinv = eye(6) - 0.5 * ad + (1 / 12) * ad2 - ...
                (1 / 720) * (ad2 * ad2) + ...
                (1 / 30240) * (ad2 * ad2 * ad2) - ...
                (1 / 1209600) * (ad2^4) + ...
                (1 / 47900160) * (ad2^5);
        end

        function [J, Jdot] = leftJacobianAndDerivative(obj, xi, xiDot)
            adXi = obj.adjointMatrix(xi);
            adXiDot = obj.adjointMatrix(xiDot);
            power = eye(6);
            powerDerivative = zeros(6);
            J = eye(6);
            Jdot = zeros(6);
            coefficient = 1;
            for order = 1:24
                powerDerivative = powerDerivative * adXi + ...
                    power * adXiDot;
                power = power * adXi;
                coefficient = coefficient / (order + 1);
                J = J + coefficient * power;
                Jdot = Jdot + coefficient * powerDerivative;
                if norm(coefficient * power, 'fro') < 1e-16 && ...
                        norm(coefficient * powerDerivative, 'fro') < 1e-16
                    break;
                end
            end
        end

        function [xiDot, xiDDot] = localKinematics(obj, xi, W, D)
            [J, ~] = obj.leftJacobianAndDerivative(xi, zeros(6, 1));
            xiDot = J \ W;
            [~, Jdot] = obj.leftJacobianAndDerivative(xi, xiDot);
            xiDDot = J \ (D - Jdot * xiDot);
        end

        function value = lastOrNaN(obj, values) %#ok<INUSL>
            if isempty(values)
                value = NaN;
            else
                value = values(end);
            end
        end

        function value = finalRelativeDecrease(obj, costs) %#ok<INUSL>
            if numel(costs) < 2
                value = NaN;
            else
                value = (costs(end - 1) - costs(end)) / ...
                    max(1, costs(end - 1));
            end
        end
    end
end
