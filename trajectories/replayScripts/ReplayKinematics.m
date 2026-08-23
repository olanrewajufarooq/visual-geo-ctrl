classdef ReplayKinematics
    %REPLAYKINEMATICS Decode replay CSVs and construct body-frame kinematics.

    methods (Static)
        function traj = processSingle(rawPath, replayId, entry)
            if ~isfile(rawPath)
                error('fth:Replay:RawFileMissing', ...
                    'Replay source file ''%s'' does not exist.', rawPath);
            end

            T = readtable(rawPath, 'VariableNamingRule', 'preserve');
            t = T.("elapsed_time");
            p = [T.("drone_x"), T.("drone_y"), T.("drone_z")];
            v_world = [T.("drone_velocity_linear_x"), ...
                T.("drone_velocity_linear_y"), T.("drone_velocity_linear_z")];
            omega_world = [T.("drone_velocity_angular_x"), ...
                T.("drone_velocity_angular_y"), T.("drone_velocity_angular_z")];
            rotCols = ["drone_rot[0]" "drone_rot[1]" "drone_rot[2]" ...
                       "drone_rot[3]" "drone_rot[4]" "drone_rot[5]" ...
                       "drone_rot[6]" "drone_rot[7]" "drone_rot[8]"];

            [R, v_b, omega_b] = ReplayKinematics.convertToBodyFrame( ...
                T, rotCols, v_world, omega_world);

            % Estimate derivatives only after the complete velocity signals have
            % been assembled. The local polynomial fit suppresses sample-level
            % MoCap/interpolation noise without introducing a causal delay.
            derivativeWindowSamples = 201;  % 402 ms at the dataset's 500 Hz rate
            v_world_dot = ReplayKinematics.differentiateSignal( ...
                v_world, t, derivativeWindowSamples);
            omega_world_dot = ReplayKinematics.differentiateSignal( ...
                omega_world, t, derivativeWindowSamples);

            [a_b, alpha_b] = ReplayKinematics.computeBodyAccelerations( ...
                R, v_b, omega_b, v_world_dot, omega_world_dot);

            traj = struct();
            traj.t = t(:);
            traj.p = p;
            traj.v_b = v_b;
            traj.a_b = a_b;
            traj.omega_b = omega_b;
            traj.alpha_b = alpha_b;
            traj.R = R;
            traj.meta = struct( ...
                'id', replayId, ...
                'source_mode', char(string(entry.source_mode)), ...
                'source_file', char(string(entry.source_file)), ...
                'sampleRateHz', ReplayKinematics.estimateSampleRate(t), ...
                'accelerationMethod', 'local-polynomial-world-derivative', ...
                'accelerationWindowSamples', derivativeWindowSamples, ...
                'tStart', t(1), ...
                'tEnd', t(end));
        end
    end

    methods (Static, Access = private)
        function [R, v_b, omega_b] = convertToBodyFrame(T, rotCols, v_world, omega_world)
            R = zeros(3, 3, height(T));
            v_b = zeros(size(v_world));
            omega_b = zeros(size(omega_world));
            for k = 1:height(T)
                % The CSV stores the body-to-world rotation in column-major form.
                R_body_to_world = reshape(T{k, rotCols}, 3, 3).';
                R_world_to_body = R_body_to_world.';
                R(:, :, k) = R_body_to_world;

                % SimRunner expects the translational and angular twist in body
                % coordinates, so convert each world-frame vector explicitly.
                v_b(k, :) = (R_world_to_body * v_world(k, :).').';
                omega_b(k, :) = (R_world_to_body * omega_world(k, :).').';
            end
        end

        function [a_b, alpha_b] = computeBodyAccelerations( ...
                R, v_b, omega_b, v_world_dot, omega_world_dot)
            % For R = body-to-world and v_b = R' * v_world:
            % d(v_b)/dt = R' * d(v_world)/dt - omega_b x v_b.
            n = size(v_b, 1);
            a_b = zeros(size(v_b));
            alpha_b = zeros(size(omega_b));
            for k = 1:n
                R_world_to_body = R(:, :, k).';
                v_body_k = v_b(k, :).';
                omega_body_k = omega_b(k, :).';
                a_b(k, :) = (R_world_to_body * v_world_dot(k, :).'- ...
                    cross(omega_body_k, v_body_k)).';
                % omega_body x omega_body is zero, so angular acceleration is
                % simply the world angular-velocity derivative rotated to body.
                alpha_b(k, :) = (R_world_to_body * omega_world_dot(k, :).').';
            end
        end

        function rate = estimateSampleRate(t)
            dt = diff(t(:));
            if isempty(dt)
                rate = 0;
                return;
            end
            rate = 1 / median(dt);
        end

        function deriv = differentiateSignal(values, t, windowSamples)
            %DIFFERENTIATESIGNAL Estimate d(values)/dt with local cubic fits.
            n = size(values, 1);
            deriv = zeros(size(values));
            if n < 2
                return;
            end

            t = t(:);
            if any(~isfinite(t)) || any(diff(t) <= 0)
                error('fth:Replay:InvalidTime', ...
                    'Replay elapsed_time must be finite and strictly increasing.');
            end

            windowSamples = min(n, max(5, round(windowSamples)));
            if mod(windowSamples, 2) == 0
                windowSamples = windowSamples - 1;
            end
            halfWindow = floor(windowSamples / 2);

            for i = 1:n
                first = max(1, i - halfWindow);
                last = min(n, i + halfWindow);
                indices = first:last;
                offsets = t(indices) - t(i);
                timeScale = max(abs(offsets));
                if timeScale == 0
                    continue;
                end

                normalizedOffsets = offsets / timeScale;
                localDegree = min(3, numel(indices) - 1);
                design = zeros(numel(indices), localDegree + 1);
                for degree = 0:localDegree
                    design(:, degree + 1) = normalizedOffsets .^ degree;
                end

                basisDerivative = zeros(localDegree + 1, 1);
                basisDerivative(2) = 1;
                % d/dt at the center = e_1' * pinv(design) * samples.
                weights = (pinv(design).' * basisDerivative) / timeScale;
                deriv(i, :) = weights.' * values(indices, :);
            end
        end
    end
end
