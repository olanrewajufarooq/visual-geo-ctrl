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
%   getEstimate, initFromCfg, computeSliding, update.

properties (Access = protected)
    g               % gravity scalar [m/s^2]
    dt              % default adaptation timestep [s]
    m_hat           % cached mass estimate [kg]
    cog_hat         % cached 3×1 CoG estimate [m]
    Iparams_hat     % cached 6×1 inertia params [Ixx; Iyy; Izz; Ixy; Ixz; Iyz]
    updateCount     % cumulative update counter
    coriolisFactor  % coriolis factorization form
    potential       % potential function object (from PotentialFactory)
    lambda          % 6×6 gain matrix for sliding variable
    spdValidCount   % count of SPD-valid parameter states
    spdInvalidCount % count of SPD-invalid parameter states
end

properties (Access = private)
    rbrObj_   % cached fth.ctrl.RigidBodyRegressor instance
end

methods (Abstract)
    pi = doUpdate(obj, dt, s, Y)
    pi = getPi(obj)
    params = getParams(obj)
    diagnostics = getDiagnostics(obj)
    setEstimatePi(obj, pi)
end

methods
    function setCoriolisFactor(obj, coriolisFactor)
        %SETCORIOLISFACTOR Store the shared Coriolis factorization object.
        if nargin < 2 || isempty(coriolisFactor)
            obj.coriolisFactor = [];
            obj.rbrObj_ = [];
            return;
        end

        if ~(isobject(coriolisFactor) && ismethod(coriolisFactor, 'getCoriolisFactor'))
            error('fth:AdaptationBase:InvalidCoriolisFactor', ...
                'coriolisFactor must provide a getCoriolisFactor(VR, I6) method.');
        end

        obj.coriolisFactor = coriolisFactor;
        obj.rbrObj_ = [];   % invalidate cached regressor
    end

    function [m_hat, cog_hat, Iparams_hat] = getEstimate(obj)
        %GETESTIMATE Return mass, CoG, and inertia estimates.
        m_hat = obj.m_hat;
        cog_hat = obj.cog_hat;
        Iparams_hat = obj.Iparams_hat;
    end

    function pi = update(obj, Hd, H, Vd, V, Ades, dt)
        %UPDATE Template: compute sliding variables and regressor, then delegate.
        % Returns pi_hat (10×1 parameter vector).
        if nargin < 6 || isempty(Ades); Ades = zeros(6,1); end
        if nargin < 7 || isempty(dt);   dt   = obj.dt;     end
        [s, VR, VRDot] = obj.computeSliding(Hd, H, Vd, V, Ades);
        Y  = obj.regressor(H, V, VR, VRDot);
        pi = obj.doUpdate(dt, s, Y);
    end

    function [s, VR, VRDot] = computeSliding(obj, Hd, H, Vd, V, Ades)
        %COMPUTESLIDING Compute sliding variable and reference velocity/acceleration.
        % Inputs:
        %   Hd   - 4×4 desired pose
        %   H    - 4×4 current pose
        %   Vd   - 6×1 desired body velocity
        %   V    - 6×1 current body velocity
        %   Ades - 6×1 desired body acceleration (optional, default zeros)
        % Outputs:
        %   s     - 6×1 sliding variable
        %   VR    - 6×1 reference body velocity
        %   VRDot - 6×1 reference body acceleration
        if nargin < 6 || isempty(Ades); Ades = zeros(6,1); end
        ts    = fth.se3.trackingState(H, Hd, V, Vd);
        eH    = obj.potential.getPotentialError(Hd, H);
        eHDot = obj.potential.getPotentialErrorDerivative(Hd, H, Vd, V);
        VR    = ts.AdInvHe * Vd - obj.lambda * eH;
        s     = ts.Ve + obj.lambda * eH;
        VRDot = -fth.se3.adV(ts.Ve) * ts.AdInvHe * Vd ...
                + ts.AdInvHe * Ades ...
                - obj.lambda * eHDot;
    end

    function Y = regressor(obj, H, V, VR, VRDot)
        %REGRESSOR 6×10 rigid-body dynamics regressor with gravity.
        %
        % Satisfies:
        %   Y*pi = I6*VRDot + C(V,I6)*VR + Wg
        %
        % V = [omega; v]  (angular then translational).
        %
        % Inputs:
        %   H      - 4×4 current pose
        %   V      - 6×1 current body velocity
        %   VR     - 6×1 reference body velocity
        %   VRDot  - 6×1 reference body acceleration

        if nargin < 4 || isempty(VR);    VR    = zeros(6,1); end
        if nargin < 5 || isempty(VRDot); VRDot = zeros(6,1); end

        if isempty(obj.rbrObj_)
            obj.rbrObj_ = obj.buildRegressorObj_();
        end

        Y = obj.rbrObj_.getRegressor(H, V, VR, VRDot);
    end
end

methods (Access = protected)
    function initFromCfg(obj, cfg)
        %INITFROMCFG Read potential and lambda from configuration struct.
        obj.potential = fth.ctrl.potential.PotentialFactory.create(cfg);
        if isfield(cfg.controller, 'lambda') && ~isempty(cfg.controller.lambda)
            lam = cfg.controller.lambda(:);
            if isscalar(lam); lam = lam * ones(6,1); end
            obj.lambda = diag(lam);
        else
            obj.lambda = zeros(6);
        end
    end

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

    function [Ades, dt, s, VR, VRDot] = parseUpdateArgs(obj, Ades, dt, s, VR, VRDot)
        %PARSEUPDATEARGS Fill default values for update() inputs.
        if nargin < 2 || isempty(Ades);   Ades   = zeros(6,1); end
        if nargin < 3 || isempty(dt);     dt     = obj.dt;      end
        if nargin < 4 || isempty(s);      s      = zeros(6,1);  end
        if nargin < 5 || isempty(VR);     VR     = zeros(6,1);  end
        if nargin < 6 || isempty(VRDot);  VRDot  = zeros(6,1);  end
    end

    function recordSPDStatus(obj, isValid)
        %RECORDSPDSTATUS Track SPD validity count.
        if isValid
            obj.spdValidCount = obj.spdValidCount + 1;
        else
            obj.spdInvalidCount = obj.spdInvalidCount + 1;
        end
    end

    function setPayloadEstimate(obj, m_payload, CoG_payload)
        %SETPAYLOADESTIMATE Shift estimates based on payload guess.
        % Inputs:
        % m_payload   - payload mass [kg].
        % CoG_payload - 3×1 payload CoG offset [m].
        pi = obj.getPi();
        pi(1)   = pi(1)   + m_payload;
        pi(2:4) = pi(2:4) + m_payload * CoG_payload(:);
        obj.setEstimatePi(pi);
    end
end

methods (Access = private)
    function rbr = buildRegressorObj_(obj)
        %BUILDREGRESSOROBJ_ Create RigidBodyRegressor from stored coriolisFactor.
        if isa(obj.coriolisFactor, 'fth.ctrl.coriolis.consistentCoriolisFactor')
            form = 'consistent';
        else
            form = 'basic';
        end
        rbr = fth.ctrl.RigidBodyRegressor(obj.g, form);
    end
end
end
