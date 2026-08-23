classdef ReplayTraj < fth.traj.TrajectoryBase
    %REPLAYTRAJ Replay a processed trajectory artifact as a reference.

    properties (Access = private)
        data
    end

    methods
        function obj = ReplayTraj(cfg)
            obj.data = ReplayProcessor.loadArtifact(cfg.traj.replay);
        end

        function [H, V, A] = generate(obj, t, ~, ~, ~)
            p = obj.interpolateRows(obj.data.p, t);
            v_b = obj.interpolateRows(obj.data.v_b, t);
            a_b = obj.interpolateRows(obj.data.a_b, t);
            omega_b = obj.interpolateRows(obj.data.omega_b, t);
            alpha_b = obj.interpolateRows(obj.data.alpha_b, t);
            R = obj.interpolateRotation(t);

            H = [R, p(:); 0 0 0 1];
            V = [omega_b(:); v_b(:)];
            A = [alpha_b(:); a_b(:)];
        end

        function reset(obj, ~, ~) %#ok<INUSD>
            %RESET Replay trajectory is stateless after construction.
        end
    end

    methods (Access = private)
        function values = interpolateRows(obj, samples, t)
            tq = min(max(t, obj.data.t(1)), obj.data.t(end));
            values = zeros(size(samples, 2), 1);
            for j = 1:size(samples, 2)
                values(j) = interp1(obj.data.t, samples(:, j), tq, 'linear');
            end
        end

        function R = interpolateRotation(obj, t)
            tq = min(max(t, obj.data.t(1)), obj.data.t(end));
            if tq <= obj.data.t(1)
                R = obj.data.R(:, :, 1);
                return;
            elseif tq >= obj.data.t(end)
                R = obj.data.R(:, :, end);
                return;
            end

            i = find(obj.data.t <= tq, 1, 'last');
            j = i + 1;
            w = (tq - obj.data.t(i)) / (obj.data.t(j) - obj.data.t(i));
            Ri = obj.data.R(:, :, i);
            Rj = obj.data.R(:, :, j);
            phi = fth.se3.skewOfMat(Ri.' * Rj);
            theta = acos(max(-1, min(1, (trace(Ri.' * Rj) - 1) / 2)));
            if theta < 1e-10
                R = Ri;
            else
                R = Ri * (eye(3) + sin(w * theta) / sin(theta) * phi + ...
                    (1 - cos(w * theta)) / (1 - cos(theta)) * (phi * phi));
            end
        end
    end
end
