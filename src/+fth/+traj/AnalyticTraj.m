classdef AnalyticTraj < fth.traj.TrajectoryBase
    %ANALYTICTRAJ Generates analytic reference trajectories on SE(3).
    %   Implements the path/time-scaling decomposition (Modern Robotics §9.1):
    %   a geometric path H(s) parameterized by s ∈ [0,1] is composed with a
    %   fifth-order time scaling s(t) to produce smooth desired motion.
    %
    %   Supported path names (cfg.traj.name):
    %     'hover'       - static hover at altitude
    %     'circle'      - planar circular loop
    %     'infinity'    - amplitude-modulated 3D figure-eight
    %     'lissajous3d' - 3D Lissajous curves with configurable amp/freq/phase
    %     'helix3d'     - helical spiral
    %     'poly3d'      - custom polynomial path (coefficients from BCs)
    %     'takeoffland' - takeoff, cruise, and landing sequence
    %
    %   Notes:
    %     - goToHoverBeforePathStarts: when true, the vehicle climbs to
    %       altitude with smooth scaling before the main path begins.
    %     - Yaw is derived from planar velocity for circle; RPY profiles are
    %       used for infinity, lissajous3d, helix3d, and poly3d.
    properties
        name
        scale
        period
        altitude
        goToHoverBeforePathStarts
        goToHoverDuration
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
        function obj = AnalyticTraj(cfg)
            %ANALYTICTRAJ Configure trajectory parameters from cfg.
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
            obj.goToHoverBeforePathStarts = cfg.traj.goToHoverBeforePathStarts;
            if isfield(cfg.traj, 'goToHoverDuration') && ~isempty(cfg.traj.goToHoverDuration)
                % Explicit duration takes priority.
                obj.goToHoverDuration = cfg.traj.goToHoverDuration;
            else
                % Fallback: 10 % of the trajectory period (legacy hoverFrac behaviour).
                hoverFrac = 0.1;
                if isfield(cfg.traj, 'hoverFrac') && ~isempty(cfg.traj.hoverFrac)
                    hoverFrac = cfg.traj.hoverFrac;
                end
                obj.goToHoverDuration = hoverFrac * obj.period;
            end

            obj.lissajousAmp = obj.ensureVec3Field(cfg.traj, 'lissajousAmp', [obj.scale; obj.scale; min(obj.scale/2, obj.altitude/2)]);
            obj.lissajousFreq = obj.ensureVec3Field(cfg.traj, 'lissajousFreq', [1; 2; 3]);
            obj.lissajousPhase = obj.ensureVec3Field(cfg.traj, 'lissajousPhase', [0; 0; 0]);

            obj.helixTurns = obj.getScalarField(cfg.traj, 'helixTurns', 1);
            obj.helixZAmp = obj.getScalarField(cfg.traj, 'helixZAmp', min(obj.scale/2, obj.altitude/2));

            obj.inf3dModAlpha = obj.getScalarField(cfg.traj, 'inf3dModAlpha', 0.3);
            obj.inf3dModBeta = obj.getScalarField(cfg.traj, 'inf3dModBeta', 0.25);

            obj.rpyAmp = obj.ensureVec3Field(cfg.traj, 'rpyAmp', (pi/180) * [10; 10; 20]);
            obj.rpyFreq = obj.ensureVec3Field(cfg.traj, 'rpyFreq', [1; 2; 3]);
            obj.rpyPhase = obj.ensureVec3Field(cfg.traj, 'rpyPhase', [0; 0; 0]);

            obj.useYawFromVelocity = ~ismember(lower(obj.name), {'lissajous3d', 'helix3d', 'infinity', 'poly3d'});
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
                % Compute poly3d coefficients from boundary conditions.
                % Each axis satisfies the 5th-order system:
                %   p(0)=p_start, dp/ds(0)=0, d²p/ds²(0)=0
                %   p(1)=p_end,   dp/ds(1)=0, d²p/ds²(1)=0
                % The BC matrix M is the same 6×6 system used for time scaling.
                p_start = [0; 0; obj.altitude];
                if isfield(cfg.traj, 'polyStart') && ~isempty(cfg.traj.polyStart)
                    p_start = cfg.traj.polyStart(:);
                end
                % Default: x sweeps 0→scale, y arches 0→0 (lateral curve),
                % z descends slightly. This avoids the straight-line degenerate
                % case that occurs when all axes use zero-velocity BCs (which
                % forces all axes to share the same normalised polynomial shape).
                z_drop = min(obj.scale / 4, obj.altitude / 4);
                p_end  = [obj.scale; 0; obj.altitude - z_drop];
                if isfield(cfg.traj, 'polyEnd') && ~isempty(cfg.traj.polyEnd)
                    p_end = cfg.traj.polyEnd(:);
                end
                M = [1 0 0  0  0  0;
                     0 1 0  0  0  0;
                     0 0 2  0  0  0;
                     1 1 1  1  1  1;
                     0 1 2  3  4  5;
                     0 0 2  6 12 20];
                obj.polyCoeff = zeros(3, 6);
                % y-axis: arch via non-zero endpoint velocities; peak ≈ 0.31*v_arch.
                v_arch = 2 * obj.scale;
                for i = 1:3
                    if i == 2
                        % Lateral arch: same y at start/end, outward velocity at both ends.
                        b = [p_start(2); v_arch; 0; p_end(2); -v_arch; 0];
                    else
                        b = [p_start(i); 0; 0; p_end(i); 0; 0];
                    end
                    a = M \ b;               % ascending coefficients
                    obj.polyCoeff(i,:) = flip(a.');  % descending for polyval
                end
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
            if obj.goToHoverBeforePathStarts
                hover_time = obj.goToHoverDuration;
                if t < hover_time
                    [s, sd, sdd] = fth.traj.TimeScaling.fifthOrder(hover_time).evaluate(t);
                    p          = [0; 0; obj.altitude * s];
                    v_inertial = [0; 0; obj.altitude * sd];
                    a_inertial = [0; 0; obj.altitude * sdd];
                    % Smoothly rotate from identity to the path's initial orientation,
                    % mirroring the position interpolation above.
                    rpy0     = obj.computePathStartRpy();
                    rpy      = rpy0 * s;
                    rpy_dot  = rpy0 * sd;
                    rpy_ddot = rpy0 * sdd;
                    R = obj.rotFromRpy(rpy);
                    H = [R, p; 0 0 0 1];
                    omega     = obj.rpyRatesToBodyOmega(rpy, rpy_dot);
                    omega_dot = obj.rpyRatesToBodyOmegaDot(rpy, rpy_dot, rpy_ddot);
                    v_des = R' * v_inertial;
                    a_des = R' * a_inertial - cross(omega, v_des);
                    V = [omega; v_des];
                    A = [omega_dot; a_des];
                    obj.lastYaw = rpy0(3);  % prime for smooth handoff when path begins
                    return;
                else
                    t = t - hover_time;
                end
            end

            T = obj.period;
            tmod = mod(t, T);
            if obj.goToHoverBeforePathStarts
                [s, sdot, sddot] = fth.traj.TimeScaling.fifthOrder(T).evaluate(tmod);
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
                    % Amplitude-modulated 3D figure-eight.
                    % x = a*sin(t)*(1+alpha*sin(2t)), similar for y, z.
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

                case 'lissajous3d'
                    % p_i(s) = amp_i * sin(2*pi*freq_i*s + phase_i)
                    % Naturally centered at (0, 0, altitude); no offset needed.
                    amp = obj.lissajousAmp;
                    freq = obj.lissajousFreq;
                    phase = obj.lissajousPhase;
                    theta = 2*pi*(freq .* s) + phase;
                    p = [amp(1)*sin(theta(1)); amp(2)*sin(theta(2)); obj.altitude + amp(3)*sin(theta(3))];
                    dp = 2*pi * [amp(1)*freq(1)*cos(theta(1)); amp(2)*freq(2)*cos(theta(2)); amp(3)*freq(3)*cos(theta(3))];
                    d2p = -(2*pi)^2 * [amp(1)*(freq(1)^2)*sin(theta(1)); amp(2)*(freq(2)^2)*sin(theta(2)); amp(3)*(freq(3)^2)*sin(theta(3))];
                    v = dp * sdot;
                    a = d2p * (sdot^2) + dp * sddot;
                    use_rpy_profile = true;

                case 'helix3d'
                    % True helix: circular xy motion + linear z ascent.
                    % x uses (cos(θ)−1) so the path starts at (0,0,altitude),
                    % matching the hover end-point and avoiding a position jump.
                    % z starts at altitude and climbs by z_amp over the full path.
                    r     = obj.scale;
                    turns = obj.helixTurns;
                    z_amp = obj.helixZAmp;
                    theta = 2*pi*turns*s;
                    p   = [r*(cos(theta)-1); r*sin(theta); obj.altitude + z_amp*s];
                    dp  = 2*pi*turns * [-r*sin(theta); r*cos(theta); 0] + [0; 0; z_amp];
                    d2p = (2*pi*turns)^2 * [-r*cos(theta); -r*sin(theta); 0];
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
                        [s1, sd1, sdd1] = fth.traj.TimeScaling.fifthOrder(t1).evaluate(tmod);
                        z = obj.altitude * s1;
                        p = [0; 0; z];
                        v = [0; 0; obj.altitude * sd1];
                        a = [0; 0; obj.altitude * sdd1];
                    elseif tmod < t2
                        [s2, sd2, sdd2] = fth.traj.TimeScaling.fifthOrder(t2 - t1).evaluate(tmod - t1);
                        x = obj.scale * s2;
                        p = [x; 0; obj.altitude];
                        v = [obj.scale * sd2; 0; 0];
                        a = [obj.scale * sdd2; 0; 0];
                    else
                        [s3, sd3, sdd3] = fth.traj.TimeScaling.fifthOrder(T - t2).evaluate(tmod - t2);
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
        function rpy0 = computePathStartRpy(obj)
            %COMPUTEPATHSTARTRPY Return the desired RPY at the start of the path (s=0).
            %   Used by the hover phase to know where to rotate towards so that
            %   orientation is continuous when the path begins.
            switch lower(obj.name)
                case {'hover', 'takeoffland'}
                    rpy0 = [0; 0; 0];

                case 'circle'
                    % Path velocity direction at s=0: dp = 2*pi*[-r*sin(0); r*cos(0); 0]
                    % = [0; 2*pi*r; 0]  →  yaw = atan2(2*pi*r, 0) = pi/2.
                    r    = obj.scale;
                    yaw0 = atan2(2*pi*r, 0);
                    rpy0 = [0; 0; yaw0];

                otherwise
                    % rpyProfile trajectories (infinity, lissajous3d, helix3d, poly3d).
                    % useYawFromVelocity is false for all of these, so rpy is purely
                    % from the profile.  At s=0 with default phase=[0,0,0] this is
                    % [0;0;0], but non-default phases are handled correctly.
                    [rpy0, ~, ~] = obj.rpyProfile(0, 1, 0);
            end
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
