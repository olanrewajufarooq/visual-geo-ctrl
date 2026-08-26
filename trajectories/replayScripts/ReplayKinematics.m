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

            options = ReplayKinematics.postprocessingOptions(entry);
            smoother = ReplayWnojSmoother(options);
            smoother = smoother.fit(t, R, p, [omega_b, v_b]);
            [R, p, omega_b, v_b, alpha_b, a_b] = ...
                ReplayKinematics.sampleSmoother(smoother);

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
                'accelerationMethod', 'wnoj', ...
                'tStart', t(1), ...
                'tEnd', t(end));
            traj.smoother = smoother;
            traj.meta.postprocessingMethod = smoother.method;
            traj.meta.postprocessingDiagnostics = smoother.diagnostics;
        end
    end

    methods (Static, Access = private)
        function options = postprocessingOptions(entry)
            options = struct();
            if isfield(entry, 'postprocessing') && ...
                    isstruct(entry.postprocessing) && ...
                    isfield(entry.postprocessing, 'wnoj') && ...
                    isstruct(entry.postprocessing.wnoj)
                options = entry.postprocessing.wnoj;
            end
        end

        function [R, p, omega_b, v_b, alpha_b, a_b] = sampleSmoother(smoother)
            outputTimes = smoother.outputTimes;
            n = numel(outputTimes);
            R = zeros(3, 3, n);
            p = zeros(n, 3);
            omega_b = zeros(n, 3);
            v_b = zeros(n, 3);
            alpha_b = zeros(n, 3);
            a_b = zeros(n, 3);
            for k = 1:n
                [H, V, A] = smoother.evaluate(outputTimes(k));
                R(:, :, k) = H(1:3, 1:3);
                p(k, :) = H(1:3, 4).';
                omega_b(k, :) = V(1:3).';
                v_b(k, :) = V(4:6).';
                alpha_b(k, :) = A(1:3).';
                a_b(k, :) = A(4:6).';
            end
        end

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

        function rate = estimateSampleRate(t)
            dt = diff(t(:));
            if isempty(dt)
                rate = 0;
                return;
            end
            rate = 1 / median(dt);
        end

    end
end
