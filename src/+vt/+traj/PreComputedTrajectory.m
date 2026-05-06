classdef PreComputedTrajectory < vt.traj.TrajectoryBase
    %PRECOMPUTEDTRAJECTORY Generates analytic reference trajectories.
    %   Supports hover, circle, infinity, Lissajous, helix, and custom
    %   polynomial paths with optional roll/pitch/yaw profiles.
    %
    %   Notes:
    %     - If startWithHover is enabled, motion begins after a hover window.
    %     - Yaw can be derived from planar velocity when enabled.
    properties
        name
        scale
        period
        altitude
        startWithHover
        hoverFrac
        lissajousAmp
        lissajousFreq
        lissajousPhase
        helixTurns
        helixZAmp
        inf3dModAlpha
        inf3dModBeta
        rpyAmp
        rpyFreq
        rpyPhase
        polyCoeff
        useYawFromVelocity
        lastYaw
    end

    methods
        function obj = PreComputedTrajectory(cfg)
            %PRECOMPUTEDTRAJECTORY Configure trajectory parameters from cfg.
            %   Input:
            %     cfg - config struct with traj fields.
            obj.name = cfg.traj.name;
            obj.scale = cfg.traj.scale;
            if isfield(cfg.traj, 'period') && ~isempty(cfg.traj.period)
                obj.period = cfg.traj.period;
            elseif isfield(cfg, 'sim') && isfield(cfg.sim, 'duration') && ~isempty(cfg.sim.duration)
                obj.period = cfg.sim.duration;
            else
                obj.period = 1;
            end
            obj.altitude = cfg.traj.altitude;
            obj.startWithHover = cfg.traj.startWithHover;
            obj.hoverFrac = 0.1;

            if isfield(cfg.traj, 'hoverFrac')
                obj.hoverFrac = cfg.traj.hoverFrac;
            end

            obj.lissajousAmp = obj.ensureVec3Field(cfg.traj, 'lissajousAmp', [obj.scale; obj.scale; min(obj.scale/2, obj.altitude/2)]);
            obj.lissajousFreq = obj.ensureVec3Field(cfg.traj, 'lissajousFreq', [1; 2; 3]);
            obj.lissajousPhase = obj.ensureVec3Field(cfg.traj, 'lissajousPhase', [0; pi/2; pi/4]);

            obj.helixTurns = obj.getScalarField(cfg.traj, 'helixTurns', 1);
            obj.helixZAmp = obj.getScalarField(cfg.traj, 'helixZAmp', min(obj.scale/2, obj.altitude/2));

            obj.inf3dModAlpha = obj.getScalarField(cfg.traj, 'inf3dModAlpha', 0.3);
            obj.inf3dModBeta = obj.getScalarField(cfg.traj, 'inf3dModBeta', 0.25);

            obj.rpyAmp = obj.ensureVec3Field(cfg.traj, 'rpyAmp', (pi/180) * [10; 10; 20]);
            obj.rpyFreq = obj.ensureVec3Field(cfg.traj, 'rpyFreq', [1; 2; 3]);
            obj.rpyPhase = obj.ensureVec3Field(cfg.traj, 'rpyPhase', [0; 0; 0]);

            obj.useYawFromVelocity = ~ismember(lower(obj.name), {'lissajous3d', 'helix3d', 'infinity3dmod', 'poly3d'});
            if isfield(cfg.traj, 'useYawFromVelocity') && ~isempty(cfg.traj.useYawFromVelocity)
                obj.useYawFromVelocity = logical(cfg.traj.useYawFromVelocity);
            end
            obj.lastYaw = 0;

            if isfield(cfg.traj, 'polyCoeff') && ~isempty(cfg.traj.polyCoeff)
                coeff = cfg.traj.polyCoeff;
                if size(coeff, 1) ~= 3 && size(coeff, 2) == 3
                    coeff = coeff.';
                end
                if size(coeff, 1) ~= 3
                    error('polyCoeff must be a 3xN matrix.');
                end
                obj.polyCoeff = coeff;
            else
                z_amp = min(obj.scale/2, obj.altitude/2);
                obj.polyCoeff = [0 16 -32 16 0 0; 32 -80 64 -16 0 0; 0 16 -32 16 0 obj.altitude];
                obj.polyCoeff(1,:) = obj.polyCoeff(1,:) * obj.scale;
                obj.polyCoeff(2,:) = obj.polyCoeff(2,:) * obj.scale;
                obj.polyCoeff(3,1:5) = obj.polyCoeff(3,1:5) * z_amp;
            end
        end

        function [H, V, A] = generate(obj, t, ~, ~, ~)
            %GENERATE Return desired pose, velocity, and acceleration.
            %   Input:
            %     t - time [s].
            %   Outputs:
            %     H - 4x4 desired pose.
            %     V - 6x1 desired body velocity.
            %     A - 6x1 desired body acceleration.
            [H, V, A] = obj.generateInternal(t);
        end

        function reset(obj, ~, ~)
            %RESET No internal state required for this trajectory.
            %   This implementation is time-parametric and stateless.
        end
    end

    methods (Access = private)
        function [H, V, A] = generateInternal(obj, t)
            %GENERATEINTERNAL Compute desired motion for a given time.
            %   Input:
            %     t - time [s].
            %   Outputs:
            %     H, V, A - desired pose, velocity, and acceleration.
            if obj.startWithHover
                hover_time = obj.hoverFrac * obj.period;
                if t < hover_time
                    [s, sd, sdd] = obj.smoothTimeScaling(t, hover_time);
                    z = obj.altitude * s;
                    p = [0; 0; z];
                    v = [0; 0; obj.altitude * sd];
                    a = [0; 0; obj.altitude * sdd];
                    yaw = 0; wyaw = 0; wyawdot = 0;
                    H = obj.poseFromYawPos(yaw, p);
                    V = [0;0;wyaw; v];
                    A = [0;0;wyawdot; a];
                    return;
                else
                    t = t - hover_time;
                end
            end

            T = obj.period;
            tmod = mod(t, T);
            if obj.startWithHover
                [s, sdot, sddot] = obj.smoothTimeScaling(tmod, T);
            else
                s = tmod / T;
                sdot = 1 / T;
                sddot = 0;
            end
            use_rpy_profile = false;

            switch lower(obj.name)
                case 'hover'
                    p = [0; 0; obj.altitude];
                    v = [0; 0; 0];
                    a = [0; 0; 0];
                    yaw = 0; wyaw = 0; wyawdot = 0;

                case 'circle'
                    % p(s) = [r*(cos(2*pi*s)-1), r*sin(2*pi*s), alt]
                    r = obj.scale;
                    theta = 2*pi*s;
                    p = [r*(cos(theta)-1); r*sin(theta); obj.altitude];
                    dp = 2*pi * [-r*sin(theta); r*cos(theta); 0];
                    d2p = (2*pi)^2 * [-r*cos(theta); -r*sin(theta); 0];
                    v = dp * sdot;
                    a = d2p * (sdot^2) + dp * sddot;
                    [yaw, wyaw, wyawdot] = obj.yawFromVelocity(v, a);

                case 'infinity'
                    % Planar figure-eight: x = a*sin(theta), y = a*sin(2*theta)/2
                    a0 = obj.scale;
                    theta = 2*pi*s;
                    p = [a0*sin(theta); a0*sin(2*theta)/2; obj.altitude];
                    dp = 2*pi * [a0*cos(theta); a0*cos(2*theta); 0];
                    d2p = (2*pi)^2 * [-a0*sin(theta); -2*a0*sin(2*theta); 0];
                    v = dp * sdot;
                    a = d2p * (sdot^2) + dp * sddot;
                    [yaw, wyaw, wyawdot] = obj.yawFromVelocity(v, a);

                case 'infinity3d'
                    % 3D figure-eight: adds z = z_amp*sin(theta) oscillation
                    a0 = obj.scale;
                    z_amp = min(obj.scale/2, obj.altitude/2);
                    theta = 2*pi*s;
                    p = [a0*sin(theta); a0*sin(2*theta)/2; obj.altitude + z_amp*sin(theta)];
                    dp = 2*pi * [a0*cos(theta); a0*cos(2*theta); z_amp*cos(theta)];
                    d2p = (2*pi)^2 * [-a0*sin(theta); -2*a0*sin(2*theta); -z_amp*sin(theta)];
                    v = dp * sdot;
                    a = d2p * (sdot^2) + dp * sddot;
                    [yaw, wyaw, wyawdot] = obj.yawFromVelocity(v, a);

                case 'lissajous3d'
                    % p_i(s) = amp_i * sin(2*pi*freq_i*s + phase_i), centered
                    amp = obj.lissajousAmp;
                    freq = obj.lissajousFreq;
                    phase = obj.lissajousPhase;
                    theta = 2*pi*(freq .* s) + phase;
                    p = [amp(1)*sin(theta(1)); amp(2)*sin(theta(2)); obj.altitude + amp(3)*sin(theta(3))];
                    p0 = [amp(1)*sin(phase(1)); amp(2)*sin(phase(2)); obj.altitude + amp(3)*sin(phase(3))];
                    p = p - p0 + [0; 0; obj.altitude];
                    dp = 2*pi * [amp(1)*freq(1)*cos(theta(1)); amp(2)*freq(2)*cos(theta(2)); amp(3)*freq(3)*cos(theta(3))];
                    d2p = -(2*pi)^2 * [amp(1)*(freq(1)^2)*sin(theta(1)); amp(2)*(freq(2)^2)*sin(theta(2)); amp(3)*(freq(3)^2)*sin(theta(3))];
                    v = dp * sdot;
                    a = d2p * (sdot^2) + dp * sddot;
                    use_rpy_profile = true;

                case 'helix3d'
                    r = obj.scale;
                    turns = obj.helixTurns;
                    z_amp = obj.helixZAmp;
                    theta = 2*pi*turns*s;
                    p = [r*cos(theta); r*sin(theta); obj.altitude + z_amp*sin(theta)];
                    p0 = [r; 0; obj.altitude];
                    p = p - p0 + [0; 0; obj.altitude];
                    dp = 2*pi*turns * [-r*sin(theta); r*cos(theta); z_amp*cos(theta)];
                    d2p = (2*pi*turns)^2 * [-r*cos(theta); -r*sin(theta); -z_amp*sin(theta)];
                    v = dp * sdot;
                    a = d2p * (sdot^2) + dp * sddot;
                    use_rpy_profile = true;

                case 'infinity3dmod'
                    % Modified 3D figure-eight with amplitude modulation:
                    % x = a*sin(t)*(1+alpha*sin(2t)), similar for y,z
                    a0 = obj.scale;
                    z_amp = min(obj.scale/2, obj.altitude/2);
                    alpha = obj.inf3dModAlpha;
                    beta = obj.inf3dModBeta;
                    theta = 2*pi*s;
                    s1 = sin(theta); c1 = cos(theta);
                    s2 = sin(2*theta); c2 = cos(2*theta);
                    x = a0 * s1 * (1 + alpha*s2);
                    y = (a0/2) * s2 * (1 + alpha*s1);
                    z = obj.altitude + z_amp * s1 * (1 + beta*s2);
                    p = [x; y; z];
                    p0 = [0; 0; obj.altitude];
                    p = p - p0 + [0; 0; obj.altitude];
                    dx_dtheta = a0 * (c1 + alpha*(c1*s2 + 2*s1*c2));
                    dy_dtheta = (a0/2) * (2*c2 + alpha*(c1*s2 + 2*s1*c2));
                    dz_dtheta = z_amp * (c1 + beta*(c1*s2 + 2*s1*c2));
                    d2x_dtheta2 = a0 * (-s1 + alpha*(-5*s1*s2 + 4*c1*c2));
                    d2y_dtheta2 = (a0/2) * (-4*s2 + alpha*(-5*s1*s2 + 4*c1*c2));
                    d2z_dtheta2 = z_amp * (-s1 + beta*(-5*s1*s2 + 4*c1*c2));
                    dp = 2*pi * [dx_dtheta; dy_dtheta; dz_dtheta];
                    d2p = (2*pi)^2 * [d2x_dtheta2; d2y_dtheta2; d2z_dtheta2];
                    v = dp * sdot;
                    a = d2p * (sdot^2) + dp * sddot;
                    use_rpy_profile = true;

                case 'poly3d'
                    coeff = obj.polyCoeff;
                    p = [polyval(coeff(1,:), s); polyval(coeff(2,:), s); polyval(coeff(3,:), s)];
                    p0 = [polyval(coeff(1,:), 0); polyval(coeff(2,:), 0); polyval(coeff(3,:), 0)];
                    p = p - p0 + [0; 0; obj.altitude];
                    dp = [polyval(polyder(coeff(1,:)), s); polyval(polyder(coeff(2,:)), s); polyval(polyder(coeff(3,:)), s)];
                    d2p = [polyval(polyder(polyder(coeff(1,:))), s); polyval(polyder(polyder(coeff(2,:))), s); polyval(polyder(polyder(coeff(3,:))), s)];
                    v = dp * sdot;
                    a = d2p * (sdot^2) + dp * sddot;
                    use_rpy_profile = true;

                case 'takeoffland'
                    t1 = 0.25*T; t2 = 0.75*T;
                    if tmod < t1
                        [s1, sd1, sdd1] = obj.smoothTimeScaling(tmod, t1);
                        z = obj.altitude * s1;
                        p = [0; 0; z];
                        v = [0; 0; obj.altitude * sd1];
                        a = [0; 0; obj.altitude * sdd1];
                    elseif tmod < t2
                        [s2, sd2, sdd2] = obj.smoothTimeScaling(tmod - t1, t2 - t1);
                        x = obj.scale * s2;
                        p = [x; 0; obj.altitude];
                        v = [obj.scale * sd2; 0; 0];
                        a = [obj.scale * sdd2; 0; 0];
                    else
                        [s3, sd3, sdd3] = obj.smoothTimeScaling(tmod - t2, T - t2);
                        z = obj.altitude * (1 - s3);
                        p = [obj.scale; 0; z];
                        v = [0; 0; -obj.altitude * sd3];
                        a = [0; 0; -obj.altitude * sdd3];
                    end
                    yaw = 0; wyaw = 0; wyawdot = 0;

                otherwise
                    error('Unknown trajectory: %s', obj.name);
            end

            if use_rpy_profile
                [rpy, rpy_dot, rpy_ddot] = obj.rpyProfile(s, sdot, sddot);
                if obj.useYawFromVelocity
                    [yaw, wyaw, wyawdot] = obj.yawFromVelocity(v, a);
                    rpy(3) = rpy(3) + yaw;
                    rpy_dot(3) = rpy_dot(3) + wyaw;
                    rpy_ddot(3) = rpy_ddot(3) + wyawdot;
                end
            else
                rpy = [0; 0; yaw];
                rpy_dot = [0; 0; wyaw];
                rpy_ddot = [0; 0; wyawdot];
            end

            R = obj.rotFromRpy(rpy);
            H = [R, p; 0 0 0 1];
            omega = obj.rpyRatesToBodyOmega(rpy, rpy_dot);
            omega_dot = obj.rpyRatesToBodyOmegaDot(rpy, rpy_dot, rpy_ddot);
            v_des = R' * v;
            a_des = R' * a - cross(omega, v_des);

            V = [omega; v_des];
            A = [omega_dot; a_des];
        end
    end

    methods (Access = private)
        function [s, sd, sdd] = smoothTimeScaling(~, t, T)
            %SMOOTHTIMESCALING Fifth-order time scaling for smooth start/stop.
            %   Inputs:
            %     t - current time.
            %     T - scaling horizon.
            %   Outputs:
            %     s, sd, sdd - scale, velocity, and acceleration.
            if T <= 0
                s = 1; sd = 0; sdd = 0;
                return;
            end
            tau = min(max(t / T, 0), 1);
            % Fifth-order polynomial s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5
            % guarantees s(0)=0, s(1)=1 with s'(0)=s'(1)=s''(0)=s''(1)=0,
            % providing C^2-smooth transitions for position, velocity, and
            % acceleration continuity.
            s = 10*tau^3 - 15*tau^4 + 6*tau^5;
            sd = (30*tau^2 - 60*tau^3 + 30*tau^4) / T;
            sdd = (60*tau - 180*tau^2 + 120*tau^3) / (T^2);
        end

        function [yaw, wyaw, wyawdot] = yawFromVelocity(obj, v, a)
            %YAWFROMVELOCITY Compute yaw and yaw rate from planar velocity.
            %   Inputs:
            %     v - 3x1 velocity.
            %     a - 3x1 acceleration.
            %   Outputs:
            %     yaw - heading angle.
            %     wyaw - yaw rate.
            %     wyawdot - yaw acceleration (set to zero).
            vx = v(1); vy = v(2);
            ax = a(1); ay = a(2);
            if hypot(vx, vy) < 1e-6
                yaw = obj.lastYaw; wyaw = 0; wyawdot = 0;
                return;
            end
            yaw = atan2(vy, vx);
            wyaw = (vx * ay - vy * ax) / (vx^2 + vy^2);
            wyawdot = 0;
            obj.lastYaw = yaw;
        end

        function H = poseFromYawPos(~, yaw, p)
            %POSEFROMYAWPOS Build a pose with yaw and position.
            %   Inputs:
            %     yaw - heading angle.
            %     p - 3x1 position.
            %   Output:
            %     H - 4x4 pose matrix.
            Rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
            H = [Rz, p; 0 0 0 1];
        end

        function v = ensureVec3Field(~, cfg, field, default)
            %ENSUREVEC3FIELD Read a field as 3x1 vector with defaults.
            %   Inputs:
            %     cfg - struct with candidate field.
            %     field - field name.
            %     default - default 3x1 value.
            %   Output:
            %     v - 3x1 vector.
            if isfield(cfg, field) && ~isempty(cfg.(field))
                v = cfg.(field);
            else
                v = default;
            end
            v = v(:);
            if numel(v) == 1
                v = repmat(v, 3, 1);
            end
            if numel(v) ~= 3
                error('%s must be a scalar or 3x1 vector.', field);
            end
        end

        function v = getScalarField(~, cfg, field, default)
            %GETSCALARFIELD Read scalar field with default.
            %   Inputs:
            %     cfg - struct with candidate field.
            %     field - field name.
            %     default - default scalar.
            %   Output:
            %     v - scalar value.
            if isfield(cfg, field) && ~isempty(cfg.(field))
                v = cfg.(field);
            else
                v = default;
            end
            if ~isscalar(v)
                error('%s must be a scalar.', field);
            end
        end

        function [rpy, rpy_dot, rpy_ddot] = rpyProfile(obj, s, sdot, sddot)
            %RPYPROFILE Generate smooth roll/pitch/yaw profiles.
            %   Inputs:
            %     s, sdot, sddot - time-scaling values.
            %   Outputs:
            %     rpy - roll/pitch/yaw angles.
            %     rpy_dot - rpy rates.
            %     rpy_ddot - rpy accelerations.
            amp = obj.rpyAmp;
            freq = obj.rpyFreq;
            phase = obj.rpyPhase;
            theta = 2*pi*(freq .* s) + phase;
            theta_dot = 2*pi*freq*sdot;
            theta_ddot = 2*pi*freq*sddot;
            rpy = amp .* sin(theta);
            rpy_dot = amp .* cos(theta) .* theta_dot;
            rpy_ddot = amp .* (-sin(theta) .* (theta_dot.^2) + cos(theta) .* theta_ddot);
        end

        function R = rotFromRpy(~, rpy)
            %ROTFROMRPY Convert roll-pitch-yaw to rotation matrix.
            %   Input:
            %     rpy - 3x1 roll/pitch/yaw.
            %   Output:
            %     R - 3x3 rotation matrix.
            phi = rpy(1); theta = rpy(2); psi = rpy(3);
            cphi = cos(phi); sphi = sin(phi);
            cth = cos(theta); sth = sin(theta);
            cps = cos(psi); sps = sin(psi);
            R = [cps*cth, cps*sth*sphi - sps*cphi, cps*sth*cphi + sps*sphi;
                 sps*cth, sps*sth*sphi + cps*cphi, sps*sth*cphi - cps*sphi;
                 -sth, cth*sphi, cth*cphi];
        end

        function omega = rpyRatesToBodyOmega(~, rpy, rpy_dot)
            %RPYRATESTOBODYOMEGA Map RPY rates to body angular velocity.
            %   Inputs:
            %     rpy - 3x1 roll/pitch/yaw.
            %     rpy_dot - 3x1 roll/pitch/yaw rates.
            %   Output:
            %     omega - 3x1 body angular velocity.
            phi = rpy(1); theta = rpy(2);
            sphi = sin(phi); cphi = cos(phi);
            sth = sin(theta); cth = cos(theta);
            T = [1, 0, -sth;
                 0, cphi, sphi*cth;
                 0, -sphi, cphi*cth];
            omega = T * rpy_dot;
        end

        function omega_dot = rpyRatesToBodyOmegaDot(~, rpy, rpy_dot, rpy_ddot)
            %RPYRATESTOBODYOMEGADOT Time derivative of body angular velocity.
            %   Inputs:
            %     rpy - 3x1 roll/pitch/yaw.
            %     rpy_dot - 3x1 rpy rates.
            %     rpy_ddot - 3x1 rpy accelerations.
            %   Output:
            %     omega_dot - 3x1 body angular acceleration.
            phi = rpy(1); theta = rpy(2);
            phi_dot = rpy_dot(1); theta_dot = rpy_dot(2);
            sphi = sin(phi); cphi = cos(phi);
            sth = sin(theta); cth = cos(theta);

            T = [1, 0, -sth;
                 0, cphi, sphi*cth;
                 0, -sphi, cphi*cth];

            dT_dphi = [0, 0, 0;
                       0, -sphi, cphi*cth;
                       0, -cphi, -sphi*cth];

            dT_dtheta = [0, 0, -cth;
                         0, 0, -sphi*sth;
                         0, 0, -cphi*sth];

            T_dot = dT_dphi * phi_dot + dT_dtheta * theta_dot;
            omega_dot = T * rpy_ddot + T_dot * rpy_dot;
        end
    end
end
