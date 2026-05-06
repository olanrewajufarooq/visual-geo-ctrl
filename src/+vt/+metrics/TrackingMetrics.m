classdef TrackingMetrics < handle
    %TRACKINGMETRICS Compute and report trajectory tracking metrics.
    %   Usage:
    %     metricsObj = vt.metrics.TrackingMetrics(logger, 'Run 1');
    %     metrics = metricsObj.computeAll();
    %     metricsObj.printReport(metrics);

    properties (SetAccess = private)
        Name
        ActualPos
        DesiredPos
        ActualRpy
        DesiredRpy
        IsAdaptive
        EstMass
        EstMassActual
        EstCoG
        EstCoGActual
        EstInertia
        EstInertiaActual
        EstIdentifiability
        PositionMetrics
        OrientationMetrics
        CombinedMetrics
        ParameterMetrics
    end

    properties (Access = private)
        IsPositionErrComputed = false
        IsOrientationErrComputed = false
        IsCombinedErrComputed = false
        IsParameterErrComputed = false
    end

    methods
        function obj = TrackingMetrics(logs, name)
            if nargin < 1
                logs = [];
            end
            if nargin < 2
                name = '';
            end

            obj.Name = name;
            if ~isempty(logs)
                obj.setLogs(logs);
            end
        end

        function setLogs(obj, logs)
            if isobject(logs) && ismethod(logs, 'finalize')
                logs = logs.finalize();
            elseif ~isstruct(logs)
                error('TrackingMetrics:InvalidLogs', 'Logs must be a Logger object or a logs struct.');
            end
            obj.ActualPos = logs.actual.pos;
            obj.DesiredPos = logs.des.pos;
            obj.ActualRpy = logs.actual.rpy;
            obj.DesiredRpy = logs.des.rpy;
            obj.EstMass = [];
            obj.EstMassActual = [];
            obj.EstCoG = [];
            obj.EstCoGActual = [];
            obj.EstInertia = [];
            obj.EstInertiaActual = [];
            obj.EstIdentifiability = struct();
            if isfield(logs, 'est') && isstruct(logs.est)
                if isfield(logs.est, 'mass') && isfield(logs.est, 'massActual')
                    obj.EstMass = logs.est.mass;
                    obj.EstMassActual = logs.est.massActual;
                end
                if isfield(logs.est, 'com') && isfield(logs.est, 'comActual')
                    obj.EstCoG = logs.est.com;
                    obj.EstCoGActual = logs.est.comActual;
                end
                if isfield(logs.est, 'inertia') && isfield(logs.est, 'inertiaActual')
                    obj.EstInertia = logs.est.inertia;
                    obj.EstInertiaActual = logs.est.inertiaActual;
                end
                if isfield(logs.est, 'identifiability') && isstruct(logs.est.identifiability)
                    obj.EstIdentifiability = logs.est.identifiability;
                end
            end
            obj.PositionMetrics = struct();
            obj.OrientationMetrics = struct();
            obj.CombinedMetrics = struct();
            obj.ParameterMetrics = struct();
            obj.IsPositionErrComputed = false;
            obj.IsOrientationErrComputed = false;
            obj.IsCombinedErrComputed = false;
            obj.IsParameterErrComputed = false;
            
            % Determine if adaptation was used
            obj.IsAdaptive = false;
            if ~isempty(obj.EstMass) || ~isempty(obj.EstCoG) || ~isempty(obj.EstInertia)
                obj.IsAdaptive = true;
            end
        end

        function metrics = computeAll(obj)
            if isempty(obj.ActualPos)
                error('TrackingMetrics:MissingLogs', 'Logs must be provided.');
            end

            metrics = struct();
            metrics.name = obj.Name;
            metrics.position = obj.computePosition();
            metrics.orientation = obj.computeOrientation();
            metrics.combined = obj.computeCombined();
            if obj.IsAdaptive
                metrics.parameters = obj.computeParameterEstimation();
            end
        end

        function metrics = computePosition(obj)
            if isempty(obj.ActualPos)
                error('TrackingMetrics:MissingLogs', 'Logs must be provided.');
            end

            if obj.IsPositionErrComputed
                metrics = obj.PositionMetrics;
                return;
            end

            posErr = obj.ActualPos - obj.DesiredPos;
            metrics = struct();
            metrics.rmse_xyz = sqrt(mean(posErr .^ 2, 1));
            metrics.rmse_total = sqrt(mean(sum(posErr .^ 2, 2)));

            range_xyz = max(obj.DesiredPos, [], 1) - min(obj.DesiredPos, [], 1);
            range_xyz(range_xyz == 0) = 1;
            metrics.nrmse_xyz = metrics.rmse_xyz ./ range_xyz;
            metrics.nrmse_total = mean(metrics.nrmse_xyz);
            metrics.tracking_score = max(0, (1 - metrics.nrmse_total) * 100);

            metrics.max_error = max(vecnorm(posErr, 2, 2));
            metrics.mean_error = mean(vecnorm(posErr, 2, 2));
            metrics.std_error = std(vecnorm(posErr, 2, 2));

            obj.PositionMetrics = metrics;
            obj.IsPositionErrComputed = true;
        end

        function metrics = computeOrientation(obj)
            if isempty(obj.ActualRpy)
                error('TrackingMetrics:MissingLogs', 'Logs must be provided.');
            end

            if obj.IsOrientationErrComputed
                metrics = obj.OrientationMetrics;
                return;
            end

            [oriErr, oriErrNorm] = obj.computeSO3Error(obj.ActualRpy, obj.DesiredRpy);
            metrics = struct();
            metrics.rmse_rpy = sqrt(mean(oriErr .^ 2, 1));
            metrics.rmse_total = sqrt(mean(oriErrNorm .^ 2));

            denom = pi;
            metrics.nrmse_rpy = metrics.rmse_rpy ./ denom;
            metrics.nrmse_total = mean(metrics.nrmse_rpy);
            metrics.tracking_score = max(0, (1 - metrics.nrmse_total) * 100);

            metrics.max_error = max(oriErrNorm);
            metrics.mean_error = mean(oriErrNorm);
            metrics.std_error = std(oriErrNorm);

            obj.OrientationMetrics = metrics;
            obj.IsOrientationErrComputed = true;
        end

        function metrics = computeCombined(obj)
            if isempty(obj.ActualPos)
                error('TrackingMetrics:MissingLogs', 'Logs must be provided.');
            end

            if obj.IsCombinedErrComputed
                metrics = obj.CombinedMetrics;
                return;
            end

            se3Err = obj.computeSE3Error(obj.ActualPos, obj.DesiredPos, obj.ActualRpy, obj.DesiredRpy);
            metrics = struct();
            metrics.rmse_total = sqrt(mean(se3Err .^ 2));
            metrics.max_error = max(se3Err);
            metrics.mean_error = mean(se3Err);
            metrics.std_error = std(se3Err);

            posMetrics = obj.computePosition();
            oriMetrics = obj.computeOrientation();
            metrics.nrmse_total = mean([posMetrics.nrmse_total, oriMetrics.nrmse_total]);
            metrics.tracking_score = max(0, (1 - metrics.nrmse_total) * 100);

            obj.CombinedMetrics = metrics;
            obj.IsCombinedErrComputed = true;
        end

        function printReport(obj)
            metrics = obj.computeAll();
            name = metrics.name;
            if isempty(name)
                name = 'Tracking';
            end

            fprintf('%s', vt.sim.ConsoleFormatter.section(sprintf('Tracking Metrics: %s', name)));

            obj.printPosition();
            obj.printOrientation();
            obj.printCombined();
            if obj.IsAdaptive && isfield(metrics, 'parameters')
                obj.printParameterEstimation();
            end

        end

        function printPosition(obj)
            metrics = obj.computePosition();

            fprintf('%s', vt.sim.ConsoleFormatter.subsection('Position Metrics'));
            fprintf('%s', vt.sim.ConsoleFormatter.kv('RMSE Total', sprintf('%.4f m', metrics.rmse_total)));
            fprintf('%s', vt.sim.ConsoleFormatter.kv('NRMSE Total', sprintf('%.4f', metrics.nrmse_total)));
            fprintf('%s', vt.sim.ConsoleFormatter.kv('Tracking Score', sprintf('%.2f %%', metrics.tracking_score)));
            fprintf('%s', vt.sim.ConsoleFormatter.vector('RMSE XYZ', metrics.rmse_xyz, '%.4f', 'm'));
            fprintf('%s', vt.sim.ConsoleFormatter.kv('Max Error', sprintf('%.4f m', metrics.max_error)));
            fprintf('%s', vt.sim.ConsoleFormatter.kv('Mean Error', sprintf('%.4f m', metrics.mean_error)));
            fprintf('%s\n', vt.sim.ConsoleFormatter.kv('Std Error', sprintf('%.4f m', metrics.std_error)));
        end

        function printOrientation(obj)
            metrics = obj.computeOrientation();

            fprintf('%s', vt.sim.ConsoleFormatter.subsection('Orientation Metrics (SO(3))'));
            fprintf('%s', vt.sim.ConsoleFormatter.kv('RMSE Total', sprintf('%.4f rad', metrics.rmse_total)));
            fprintf('%s', vt.sim.ConsoleFormatter.kv('NRMSE Total', sprintf('%.4f', metrics.nrmse_total)));
            fprintf('%s', vt.sim.ConsoleFormatter.kv('Tracking Score', sprintf('%.2f %%', metrics.tracking_score)));
            fprintf('%s', vt.sim.ConsoleFormatter.vector('RMSE RPY', metrics.rmse_rpy, '%.4f', 'rad'));
            fprintf('%s', vt.sim.ConsoleFormatter.kv('Max Error', sprintf('%.4f rad', metrics.max_error)));
            fprintf('%s', vt.sim.ConsoleFormatter.kv('Mean Error', sprintf('%.4f rad', metrics.mean_error)));
            fprintf('%s\n', vt.sim.ConsoleFormatter.kv('Std Error', sprintf('%.4f rad', metrics.std_error)));
        end

        function printCombined(obj)
            metrics = obj.computeCombined();

            fprintf('%s', vt.sim.ConsoleFormatter.subsection('Combined Pose Metrics (SE(3))'));
            fprintf('%s', vt.sim.ConsoleFormatter.kv('RMSE Total', sprintf('%.4f', metrics.rmse_total)));
            fprintf('%s', vt.sim.ConsoleFormatter.kv('NRMSE Total', sprintf('%.4f', metrics.nrmse_total)));
            fprintf('%s', vt.sim.ConsoleFormatter.kv('Tracking Score', sprintf('%.2f %%', metrics.tracking_score)));
            fprintf('%s', vt.sim.ConsoleFormatter.kv('Max Error', sprintf('%.4f', metrics.max_error)));
            fprintf('%s', vt.sim.ConsoleFormatter.kv('Mean Error', sprintf('%.4f', metrics.mean_error)));
            fprintf('%s\n', vt.sim.ConsoleFormatter.kv('Std Error', sprintf('%.4f', metrics.std_error)));
        end

        function printParameterEstimation(obj)
            metrics = obj.computeParameterEstimation();

            fprintf('%s', vt.sim.ConsoleFormatter.subsection('Parameter Estimation Metrics (Diagnostic Only)'));
            fprintf('%s', vt.sim.ConsoleFormatter.note('Convergence requires persistent excitation.'));
            ident = metrics.identifiability;
            fprintf('%s', vt.sim.ConsoleFormatter.kv('Mass RMSE', sprintf('%.4f kg', metrics.mass.rmse)));
            fprintf('%s', vt.sim.ConsoleFormatter.kv('Mass NRMSE', sprintf('%.4f', metrics.mass.nrmse)));
            fprintf('%s', vt.sim.ConsoleFormatter.kv('Mass Tracking Score', sprintf('%.2f %%', metrics.mass.tracking_score)));
            fprintf('%s', vt.sim.ConsoleFormatter.kv('Mass Identification Metric', sprintf('%.4f', ident.mass.sigma_min)));
            fprintf('%s', vt.sim.ConsoleFormatter.kv('Mass Identification Score', sprintf('%.2f %%', ident.mass.score)));

            fprintf('%s', vt.sim.ConsoleFormatter.vector('CoG RMSE', metrics.cog.rmse_xyz, '%.4f', 'm'));
            fprintf('%s', vt.sim.ConsoleFormatter.kv('CoG RMSE Total', sprintf('%.4f m', metrics.cog.rmse_total)));
            fprintf('%s', vt.sim.ConsoleFormatter.kv('CoG NRMSE', sprintf('%.4f', metrics.cog.nrmse_total)));
            fprintf('%s', vt.sim.ConsoleFormatter.kv('CoG Tracking Score', sprintf('%.2f %%', metrics.cog.tracking_score)));
            fprintf('%s', vt.sim.ConsoleFormatter.kv('CoG Identification Metric', sprintf('%.4f', ident.mcog.sigma_min)));
            fprintf('%s', vt.sim.ConsoleFormatter.kv('CoG Identification Score', sprintf('%.2f %%', ident.mcog.score)));

            fprintf('%s', vt.sim.ConsoleFormatter.vector('Inertia RMSE', metrics.inertia.rmse_params, '%.4f'));
            fprintf('%s', vt.sim.ConsoleFormatter.kv('Inertia RMSE Total', sprintf('%.4f', metrics.inertia.rmse_total)));
            fprintf('%s', vt.sim.ConsoleFormatter.kv('Inertia NRMSE', sprintf('%.4f', metrics.inertia.nrmse_total)));
            fprintf('%s', vt.sim.ConsoleFormatter.kv('Inertia Tracking Score', sprintf('%.2f %%', metrics.inertia.tracking_score)));
            fprintf('%s', vt.sim.ConsoleFormatter.kv('Inertia Identification Metric', sprintf('%.4f', ident.inertia.sigma_min)));
            fprintf('%s\n', vt.sim.ConsoleFormatter.kv('Inertia Identification Score', sprintf('%.2f %%', ident.inertia.score)));
        end
    end

    methods (Access = private)
        function [oriErr, oriErrNorm] = computeSO3Error(obj, actualRpy, desiredRpy)
            n = size(actualRpy, 1);
            oriErr = zeros(n, 3);
            oriErrNorm = zeros(n, 1);

            for i = 1:n
                R = obj.rpyToRotm(actualRpy(i, :));
                Rd = obj.rpyToRotm(desiredRpy(i, :));
                Rerr = Rd' * R;

                T = eye(4);
                T(1:3, 1:3) = Rerr;
                zeta = vt.se3.logSE3(T);
                oriErr(i, :) = zeta(1:3).';
                oriErrNorm(i, 1) = norm(vt.se3.hat3(zeta(1:3)), 'fro');
            end
        end

        function se3Err = computeSE3Error(obj, actualPos, desiredPos, actualRpy, desiredRpy)
            n = size(actualPos, 1);
            se3Err = zeros(n, 1);

            for i = 1:n
                H = obj.buildSE3(actualPos(i, :), actualRpy(i, :));
                Hd = obj.buildSE3(desiredPos(i, :), desiredRpy(i, :));
                He = vt.se3.invSE3(Hd) * H;
                zeta = vt.se3.logSE3(He);
                se3mat = vt.se3.hat6(zeta);
                se3Err(i, 1) = norm(se3mat, 'fro');
            end
        end

        function H = buildSE3(obj, pos, rpy)
            R = obj.rpyToRotm(rpy);
            H = eye(4);
            H(1:3, 1:3) = R;
            H(1:3, 4) = pos(:);
        end

        function R = rpyToRotm(~, rpy)
            R = vt.utils.rpy2rotm(rpy(:));
        end

        function metrics = computeParameterEstimation(obj)
            if obj.IsParameterErrComputed
                metrics = obj.ParameterMetrics;
                return;
            end

            metrics = struct();
            metrics.mass = struct('rmse', NaN, 'nrmse', NaN, 'tracking_score', NaN);
            metrics.cog = struct('rmse_xyz', [NaN NaN NaN], 'rmse_total', NaN, ...
                'nrmse_xyz', [NaN NaN NaN], 'nrmse_total', NaN, 'tracking_score', NaN);
            metrics.inertia = struct('rmse_params', [NaN NaN NaN NaN NaN NaN], 'rmse_total', NaN, ...
                'nrmse_params', [NaN NaN NaN NaN NaN NaN], 'nrmse_total', NaN, 'tracking_score', NaN);
            metrics.identifiability = struct( ...
                'mass', obj.defaultIdentifiabilityMetric(1), ...
                'mcog', obj.defaultIdentifiabilityMetric(3), ...
                'inertia', obj.defaultIdentifiabilityMetric(6), ...
                'update_count', NaN);

            if ~isempty(obj.EstMass) && ~isempty(obj.EstMassActual)
                err = obj.EstMass - obj.EstMassActual;
                rmse = sqrt(mean(err .^ 2));
                range_val = max(obj.EstMassActual) - min(obj.EstMassActual);
                if range_val == 0
                    range_val = 1;
                end
                nrmse = rmse / range_val;
                metrics.mass = struct('rmse', rmse, 'nrmse', nrmse, ...
                    'tracking_score', max(0, (1 - nrmse) * 100));
            end

            if ~isempty(obj.EstCoG) && ~isempty(obj.EstCoGActual)
                err = obj.EstCoG - obj.EstCoGActual;
                rmse_xyz = sqrt(mean(err .^ 2, 1));
                rmse_total = sqrt(mean(sum(err .^ 2, 2)));
                range_xyz = max(obj.EstCoGActual, [], 1) - min(obj.EstCoGActual, [], 1);
                range_xyz(range_xyz == 0) = 1;
                nrmse_xyz = rmse_xyz ./ range_xyz;
                nrmse_total = mean(nrmse_xyz);
                metrics.cog = struct('rmse_xyz', rmse_xyz, 'rmse_total', rmse_total, ...
                    'nrmse_xyz', nrmse_xyz, 'nrmse_total', nrmse_total, ...
                    'tracking_score', max(0, (1 - nrmse_total) * 100));
            end

            if ~isempty(obj.EstInertia) && ~isempty(obj.EstInertiaActual)
                err = obj.EstInertia - obj.EstInertiaActual;
                rmse_params = sqrt(mean(err .^ 2, 1));
                rmse_total = sqrt(mean(sum(err .^ 2, 2)));
                range_params = max(obj.EstInertiaActual, [], 1) - min(obj.EstInertiaActual, [], 1);
                range_params(range_params == 0) = 1;
                nrmse_params = rmse_params ./ range_params;
                nrmse_total = mean(nrmse_params);
                metrics.inertia = struct('rmse_params', rmse_params, 'rmse_total', rmse_total, ...
                    'nrmse_params', nrmse_params, 'nrmse_total', nrmse_total, ...
                    'tracking_score', max(0, (1 - nrmse_total) * 100));
            end

            if isfield(obj.EstIdentifiability, 'infoMatrix') && ~isempty(obj.EstIdentifiability.infoMatrix)
                infoMatrix = obj.EstIdentifiability.infoMatrix;
                metrics.identifiability = obj.computeIdentifiabilityMetrics(infoMatrix);
                metrics.identifiability.update_count = obj.readIdentifiabilityUpdateCount();
            end

            obj.ParameterMetrics = metrics;
            obj.IsParameterErrComputed = true;
        end

        function metrics = computeIdentifiabilityMetrics(obj, infoMatrix)
            F = infoMatrix;
            F = (F + F.') / 2;
            diagF = diag(F);
            invSqrtDiag = zeros(size(diagF));
            active = diagF > eps;
            invSqrtDiag(active) = 1 ./ sqrt(diagF(active));
            G = diag(invSqrtDiag) * F * diag(invSqrtDiag);
            G = (G + G.') / 2;

            metrics = struct();
            metrics.mass = obj.groupIdentifiabilityMetric(G, 7);
            metrics.mcog = obj.groupIdentifiabilityMetric(G, 8:10);
            metrics.inertia = obj.groupIdentifiabilityMetric(G, 1:6);
        end

        function metric = groupIdentifiabilityMetric(~, G, groupIdx)
            groupIdx = groupIdx(:).';
            otherIdx = setdiff(1:size(G, 1), groupIdx);
            Ggg = G(groupIdx, groupIdx);
            if isempty(otherIdx)
                Sg = Ggg;
            else
                Ggr = G(groupIdx, otherIdx);
                Grr = G(otherIdx, otherIdx);
                Sg = Ggg - Ggr * pinv(Grr) * Ggr.';
            end
            Sg = (Sg + Sg.') / 2;

            singularValues = svd(Sg);
            if isempty(singularValues)
                sigmaMin = NaN;
                r = 0;
            else
                sigmaMin = singularValues(end);
                tol = max(size(Sg)) * eps(max([singularValues(:); 1]));
                r = sum(singularValues > tol);
            end

            metric = struct( ...
                'score', 100 * min(max(sigmaMin, 0), 1), ...
                'sigma_min', sigmaMin, ...
                'rank', r, ...
                'dimension', numel(groupIdx));
        end

        function metric = defaultIdentifiabilityMetric(~, dimension)
            metric = struct('score', NaN, 'sigma_min', NaN, 'rank', NaN, 'dimension', dimension);
        end

        function count = readIdentifiabilityUpdateCount(obj)
            count = NaN;
            if isfield(obj.EstIdentifiability, 'updateCount') && ~isempty(obj.EstIdentifiability.updateCount)
                count = obj.EstIdentifiability.updateCount;
            end
        end
    end
end
