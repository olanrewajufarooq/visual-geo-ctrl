classdef (Abstract) AdaptationBase < handle
%ADAPTATIONBASE Interface for parameter adaptation laws.
% Implementations estimate mass, CoG, and inertia for the controller.
%
% Parameter vector convention (pi, 10×1):
% pi = [m, hx, hy, hz, Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
% where h = m * CoG is the first moment of mass.
%
% Output struct fields:
% m, CoG, Iparams (legacy ordering [Ixx Iyy Izz Ixy Iyz Ixz]), I6.
%
% Concrete methods provided to all subclasses:
%   unpackPi, packPi, regressor, parseUpdateArgs, setPayloadEstimate,
%   getEstimate.

properties (Access = protected)
    g           % gravity scalar [m/s^2]
    dt          % default adaptation timestep [s]
    m_hat       % cached mass estimate [kg]
    cog_hat     % cached 3×1 CoG estimate [m]
    Iparams_hat % cached 6×1 inertia params (legacy ordering)
    updateCount % cumulative update counter
    I_basis     % cell array of 10 basis matrices for inertia parameterization
    G_basis     % cell array of 4 gravity basis matrices
end

methods (Abstract)
    params = update(obj, Hd, H, Vd, V, Ades, dt, s, VR, VRDot)
    params = getParams(obj)
    diagnostics = getDiagnostics(obj)
    setEstimatePi(obj, pi)
end

methods
    function [m_hat, cog_hat, Iparams_hat] = getEstimate(obj)
        %GETESTIMATE Return mass, CoG, and inertia estimates.
        m_hat = obj.m_hat;
        cog_hat = obj.cog_hat;
        Iparams_hat = obj.Iparams_hat;
    end
end

methods (Access = protected)
    function [m_hat, cog_hat, Iparams_hat] = unpackPi(~, pi)
        %UNPACKPI Convert pi vector into physical parameter caches.
        % Input:
        % pi - 10×1 vector [m; hx; hy; hz; Ixx; Iyy; Izz; Ixy; Ixz; Iyz].
        % Outputs:
        % m_hat       - mass estimate.
        % cog_hat     - 3×1 CoG estimate.
        % Iparams_hat - 6×1 inertia params in legacy ordering
        %               [Ixx; Iyy; Izz; Ixy; Iyz; Ixz].
        p = pi(:);
        m_hat = max(p(1), 1e-9);
        cog_hat = p(2:4) / m_hat;
        Iparams_hat = p([5; 6; 7; 8; 10; 9]);
    end

    function pi = packPi(~, m, CoG, Iparams_legacy)
        %PACKPI Build pi from physical parameters.
        % Iparams_legacy ordering: [Ixx; Iyy; Izz; Ixy; Iyz; Ixz]
        % pi ordering: [m; hx; hy; hz; Ixx; Iyy; Izz; Ixy; Ixz; Iyz]
        I = Iparams_legacy(:);
        I_pi = [I(1); I(2); I(3); I(4); I(6); I(5)];
        pi = [m; m * CoG(:); I_pi];
    end

    function constructBases(obj)
        %CONSTRUCTBASES Build inertia and gravity basis matrices.
        %   The generalized inertia I6 is parameterized linearly:
        %     I6 = sum_{i=1}^{10} pi_i * I_basis{i}
        %   where pi = [m; hx; hy; hz; Ixx; Iyy; Izz; Ixy; Ixz; Iyz]
        %   This enables the regressor-based adaptation law.
        %
        %   Basis layout (matching pi ordering):
        %     1:    Mass m - scales the translational block (I6(4:6,4:6)).
        %     2-4:  Mass-weighted CoG (m*cx, m*cy, m*cz) - couples
        %           rotation and translation via hat(e_i) blocks.
        %     5-7:  Principal inertias (Ixx, Iyy, Izz) - diagonal of
        %           the rotational block.
        %     8-10: Cross inertias (Ixy, Ixz, Iyz) - off-diagonal
        %           symmetric entries of the rotational block.
        %
        %   Gravity bases G_basis{1..4} decompose the gravity wrench
        %   into the parameter ordering (mass, then CoG axes).
        obj.I_basis = cell(10, 1);
        obj.G_basis = cell(4, 1);

        % Basis 1: mass (translational inertia)
        Gi = zeros(6, 6);
        Gi(4:6, 4:6) = eye(3);
        obj.I_basis{1} = Gi;

        % Bases 2-4: mass*CoG coupling (rotation-translation cross terms)
        for ax = 1:3
            e = zeros(3, 1);
            e(ax) = 1;
            S = fth.se3.hat3(e);
            Gi = zeros(6, 6);
            Gi(1:3, 4:6) = S;
            Gi(4:6, 1:3) = -S;
            obj.I_basis{1 + ax} = Gi;
        end

        % Bases 5-7: principal moments of inertia (Ixx, Iyy, Izz)
        for k = 1:3
            E = zeros(3, 3);
            E(k, k) = 1;
            Gi = zeros(6, 6);
            Gi(1:3, 1:3) = E;
            obj.I_basis{4 + k} = Gi;
        end

        % Bases 8-10: cross (off-diagonal) inertias (Ixy, Ixz, Iyz)
        pairs = [1 2; 1 3; 2 3];
        for idx = 1:3
            i = pairs(idx, 1);
            j = pairs(idx, 2);
            E = zeros(3, 3);
            E(i, j) = 1;
            E(j, i) = 1;
            Gi = zeros(6, 6);
            Gi(1:3, 1:3) = E;
            obj.I_basis{7 + idx} = Gi;
        end

        % Gravity bases: mass contribution (force in inertial z-direction)
        G1 = zeros(6, 6);
        G1(4:6, 4:6) = eye(3);
        obj.G_basis{1} = G1;

        % Gravity bases: CoG coupling to gravity torque (3 axes)
        for ax = 1:3
            e = zeros(3, 1);
            e(ax) = 1;
            S = fth.se3.hat3(e);
            Gi = zeros(6, 6);
            Gi(1:3, 1:3) = S;
            obj.G_basis{1 + ax} = Gi;
        end
    end

    function Y = regressor(obj, H, V, VR, VRDot)
        %REGRESSOR 6×10 reference regressor with gravity.
        % Y satisfies the identity:
        %   Y * pi = I6*VRDot + ad_VR^T*I6*VR + W_gravity
        % where VR/VRDot are reference trajectories and W_gravity is the
        % gravity wrench in body-fixed coordinates.
        %
        % Inputs:
        %   H     - 4×4 current pose (for gravity rotation).
        %   V     - 6×1 current body velocity (unused, for compatibility).
        %   VR    - 6×1 reference body velocity.
        %   VRDot - 6×1 reference body acceleration.
        % Output:
        %   Y - 6×10 regressor matrix.

        if nargin < 4 || isempty(VR)
            VR = zeros(6, 1);
        end
        if nargin < 5 || isempty(VRDot)
            VRDot = zeros(6, 1);
        end

        % Gravity vector in body-fixed frame
        R = H(1:3, 1:3);
        g_body = R' * [0; 0; obj.g];
        gvec = [g_body; g_body];  % 6×1 gravity vector (same in both blocks)

        % Build regressor column-by-column using basis matrices
        Y = zeros(6, 10);
        for i = 1:10
            % Inertia contribution: ad_VR^T * B_i * VR
            term = fth.se3.adV(VR)' * obj.I_basis{i} * VR;

            % Acceleration contribution: -B_i * VRDot
            term = term - obj.I_basis{i} * VRDot;

            % Gravity contribution for mass/CoG parameters (1-4)
            if i <= 4
                term = term + obj.G_basis{i} * gvec;
            end

            Y(:, i) = term;
        end
    end

    function [Ades, dt, s, VR, VRDot] = parseUpdateArgs(obj, Ades, dt, s, VR, VRDot)
        %PARSEUPDATEARGS Fill default values for update() inputs.
        if nargin < 2 || isempty(Ades);   Ades   = zeros(6,1); end
        if nargin < 3 || isempty(dt);     dt     = obj.dt;      end
        if nargin < 4 || isempty(s);      s      = zeros(6,1);  end
        if nargin < 5 || isempty(VR);     VR     = zeros(6,1);  end
        if nargin < 6 || isempty(VRDot);  VRDot  = zeros(6,1);  end
    end

    function setPayloadEstimate(obj, m_payload, CoG_payload)
        %SETPAYLOADESTIMATE Shift estimates based on payload guess.
        % Inputs:
        % m_payload   - payload mass [kg].
        % CoG_payload - 3×1 payload CoG offset [m].
        % Subclasses that do not store pi_hat must override this method.
        obj.pi_hat(1)   = obj.pi_hat(1)   + m_payload;
        obj.pi_hat(2:4) = obj.pi_hat(2:4) + m_payload * CoG_payload(:);
        obj.updateEstimates();
    end
end
end
