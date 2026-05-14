classdef (Abstract) AdaptationBase < handle
%ADAPTATIONBASE Interface for parameter adaptation laws.
% Implementations estimate mass, CoG, and inertia for the controller.
%
% Parameter vector convention (pi, 10×1):
% pi = [m, hx, hy, hz, Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
% where h = m * CoG is the first moment of mass.
%
% Output struct fields:
% m, CoG, Iparams [Ixx Iyy Izz Ixy Ixz Iyz], I6.
%
% Concrete methods provided to all subclasses:
%   unpackPi, packPi, regressor, parseUpdateArgs, setPayloadEstimate,
%   getEstimate.

properties (Access = protected)
    g               % gravity scalar [m/s^2]
    dt              % default adaptation timestep [s]
    m_hat           % cached mass estimate [kg]
    cog_hat         % cached 3×1 CoG estimate [m]
    Iparams_hat     % cached 6×1 inertia params [Ixx; Iyy; Izz; Ixy; Ixz; Iyz]
    updateCount     % cumulative update counter
    I_basis         % cell array of 10 basis matrices for inertia parameterization
    G_basis         % cell array of 4 gravity basis matrices
    coriolisFactor  % coriolis factorization form
end

methods (Abstract)
    params = update(obj, Hd, H, Vd, V, Ades, dt, s, VR, VRDot)
    params = getParams(obj)
    diagnostics = getDiagnostics(obj)
    setEstimatePi(obj, pi)
end

methods
    function setCoriolisFactor(obj, coriolisFactor)
        %SETCORIOLISFACTOR Store the shared Coriolis factorization object.
        if nargin < 2 || isempty(coriolisFactor)
            obj.coriolisFactor = [];
            return;
        end

        if ~(isobject(coriolisFactor) && ismethod(coriolisFactor, 'getCoriolisFactor'))
            error('fth:AdaptationBase:InvalidCoriolisFactor', ...
                'coriolisFactor must provide a getCoriolisFactor(VR, I6) method.');
        end

        obj.coriolisFactor = coriolisFactor;
    end

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
        % Iparams_hat - 6×1 inertia params [Ixx; Iyy; Izz; Ixy; Ixz; Iyz].
        p = pi(:);
        m_hat = max(p(1), 1e-9);
        cog_hat = p(2:4) / m_hat;
        Iparams_hat = p(5:10);
    end

    function pi = packPi(~, m, CoG, Iparams)
        %PACKPI Build pi from physical parameters.
        % Iparams ordering: [Ixx; Iyy; Izz; Ixy; Ixz; Iyz]
        % pi ordering: [m; hx; hy; hz; Ixx; Iyy; Izz; Ixy; Ixz; Iyz]
        pi = [m; m * CoG(:); Iparams(:)];
    end

    function Y = regressor(obj, H, V, VR, VRDot)
        %REGRESSOR 6x10 rigid-body dynamics regressor with gravity.
        %
        % Satisfies:
        %
        %   Y*pi = I6*VRDot + C(VR,I6)*VR + Wg
        %
        % where:
        %   pi = [m; h; Jparams]
        %
        % Inputs:
        %   H      - current pose
        %   V      - current velocity (unused)
        %   VR     - reference velocity
        %   VRDot  - reference acceleration

        %#ok<INUSD>

        if nargin < 4 || isempty(VR)
            VR = zeros(6,1);
        end

        if nargin < 5 || isempty(VRDot)
            VRDot = zeros(6,1);
        end

        if isempty(obj.coriolisFactor)
            obj.coriolisFactor = fth.ctrl.coriolis.basicCoriolisFactor();
        end

        Y = fth.utils.regressor( ...
                H, VR, VRDot, obj.g, obj.coriolisFactor);

        % %--------------------------------------------------------------
        % % Rigid-body dynamics regressor
        % %--------------------------------------------------------------

        % Ydyn = fth.utils.RBDynamics.regressor_rb_dynamics(VR, VRDot);

        % %--------------------------------------------------------------
        % % Gravity regressor
        % %--------------------------------------------------------------

        % R = H(1:3,1:3);

        % g_body = R' * [0;0;obj.g];

        % Yg = zeros(6,10);

        % % mass contribution
        % %
        % % force = m*g_body
        % %
        % Yg(4:6,1) = g_body;

        % % first moment contribution
        % %
        % % torque = h x g_body = -hat(g_body)*h
        % %
        % Yg(1:3,2:4) = -fth.se3.hat3(g_body);

        % % full regressor
        % Y = Ydyn + Yg;
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
