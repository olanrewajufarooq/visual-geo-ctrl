classdef EuclideanAdaptation < vt.ctrl.adapt.AdaptationBase
    %EUCLIDEANADAPTATION Gradient adaptation in Euclidean parameter space.
    %   Estimates mass, CoG, and inertia parameters using a linear regressor.
    %
    %   The 10x1 parameter vector is:
    %     theta = [I_xx, I_yy, I_zz, I_xy, I_yz, I_xz, m, m*cx, m*cy, m*cz]
    %   where (cx,cy,cz) is the CoG offset.
    %
    %   Adaptation law (Euclidean gradient descent):
    %     theta_hat_dot = Gamma * Y(q,V,Vd,Ad)^T * V_e
    %   where Y is the 6x10 dynamic regressor and V_e = V - Ad^{-1}(H_e)*Vd
    %   is the body-velocity tracking error.
    %
    %   The regressor Y satisfies:  ad_V^T*I6*V + W_gravity - I6*a_ref = Y*theta
    %   where a_ref = Ad^{-1}(H_e)*(Vd_dot + ad_Vd*Ad(H_e)*V_e).
    %   The adaptation drives theta_hat toward the true parameters.
    properties (Access = private)
        g
        theta_hat
        Gamma
        I_basis
        G_basis
        dt
        m_hat
        cog_hat
        Iparams_hat
        infoMatrix
        updateCount
    end

    methods
        function obj = EuclideanAdaptation(cfg)
            %EUCLIDEANADAPTATION Initialize estimates and gain matrix.
            %   Input:
            %     cfg - configuration with vehicle and controller fields.
            %   Output:
            %     obj - EuclideanAdaptation instance.
            obj.g = cfg.vehicle.g;
            obj.dt = cfg.sim.adaptation_dt;

            I_params = cfg.vehicle.I_params(:);
            m = cfg.vehicle.m;
            CoG = cfg.vehicle.CoG(:);
            obj.theta_hat = [I_params(1:6); m; (m * CoG(:))];

            obj.Gamma = obj.buildGamma(cfg.controller.Gamma);
            obj.constructBases();
            obj.infoMatrix = zeros(10, 10);
            obj.updateCount = 0;
            obj.updateEstimates();
        end

        function params = update(obj, Hd, H, Vd, V, Ades, dt)
            %UPDATE Step parameter estimates using the regressor.
            %   Inputs:
            %     Hd, H - desired and actual pose.
            %     Vd, V - desired and actual body velocity.
            %     Ades - desired body acceleration (optional).
            %     dt - timestep [s] (optional).
            %   Output:
            %     params - struct with updated parameters.
            if nargin < 7 || isempty(dt)
                dt = obj.dt;
            end
            if nargin < 6 || isempty(Ades)
                Ades = zeros(6,1);
            end

            H_err = vt.se3.invSE3(Hd) * H;
            Ad_inv_err = vt.se3.Ad_inv(H_err);
            V_e = V - Ad_inv_err * Vd;

            Y = obj.regressor(H_err, H, V, Vd, Ades, Ad_inv_err, V_e);
            obj.infoMatrix = obj.infoMatrix + dt * (Y.' * Y);
            obj.updateCount = obj.updateCount + 1;
            obj.theta_hat = obj.theta_hat + obj.Gamma * (Y.' * V_e) * dt;
            obj.updateEstimates();

            params = obj.getParams();
        end

        function params = getParams(obj)
            %GETPARAMS Return current estimated parameters.
            %   Output:
            %     params - struct with m, CoG, Iparams, I6.
            params = struct('m', obj.m_hat, 'CoG', obj.cog_hat, ...
                'Iparams', obj.Iparams_hat, 'I6', vt.utils.getGeneralizedInertia(obj.m_hat, obj.Iparams_hat, obj.cog_hat));
        end

        function [m_hat, cog_hat, Iparams_hat] = getEstimate(obj)
            %GETESTIMATE Return mass, CoG, and inertia estimates.
            %   Outputs:
            %     m_hat - mass estimate.
            %     cog_hat - 3x1 CoG estimate.
            %     Iparams_hat - inertia parameter estimate.
            m_hat = obj.m_hat;
            cog_hat = obj.cog_hat;
            Iparams_hat = obj.Iparams_hat;
        end

        function diagnostics = getDiagnostics(obj)
            %GETDIAGNOSTICS Return cumulative regressor diagnostics.
            diagnostics = struct( ...
                'infoMatrix', obj.infoMatrix, ...
                'updateCount', obj.updateCount);
        end

        function setPayloadEstimate(obj, m_payload, CoG_payload)
            %SETPAYLOADESTIMATE Shift estimates based on payload guess.
            %   Inputs:
            %     m_payload - payload mass [kg].
            %     CoG_payload - 3x1 payload CoG offset [m].
            if isempty(obj.theta_hat) || numel(obj.theta_hat) < 10
                return;
            end
            m = obj.theta_hat(7) + m_payload;
            mcog = obj.theta_hat(8:10) + m_payload * CoG_payload(:);
            obj.theta_hat(7) = m;
            obj.theta_hat(8:10) = mcog;
            obj.updateEstimates();
        end

        function setEstimateTheta(obj, theta)
            %SETESTIMATETHETA Replace the adaptive estimate state.
            validateattributes(theta, {'numeric'}, {'vector', 'numel', 10});
            obj.theta_hat = theta(:);
            obj.updateEstimates();
        end
    end

    methods (Access = private)
        function updateEstimates(obj)
            %UPDATEESTIMATES Unpack parameter vector into estimates.
            [obj.m_hat, obj.cog_hat, obj.Iparams_hat] = obj.unpackTheta(obj.theta_hat);
        end

        function Gamma = buildGamma(~, gamma)
            %BUILDGAMMA Build a diagonal gain matrix from a 10x1 vector.
            %   Input:
            %     gamma - 10x1 adaptive gain vector.
            %   Output:
            %     Gamma - 10x10 gain matrix.
            validateattributes(gamma, {'numeric'}, {'vector','numel',10});
            Gamma = diag(gamma(:));
        end

        function constructBases(obj)
            %CONSTRUCTBASES Build inertia and gravity basis matrices.
            %   The generalized inertia I6 is parameterized as:
            %     I6 = sum_{i=1}^{10} theta_i * I_basis{i}
            %   This linearity enables the regressor-based adaptation law.
            %
            %   Basis layout:
            %     1-3:  Principal inertias (I_xx, I_yy, I_zz) — diagonal of
            %           the rotational block.
            %     4-6:  Cross inertias (I_xy, I_yz, I_xz) — off-diagonal
            %           symmetric entries of the rotational block.
            %     7:    Mass m — scales the translational block (I6(4:6,4:6)).
            %     8-10: Mass-weighted CoG (m*cx, m*cy, m*cz) — couples
            %           rotation and translation via hat(e_i) blocks.
            %
            %   Gravity basis G_basis{1..4} decomposes the gravity wrench
            %   into the same parameter ordering (mass, then CoG axes).
            obj.I_basis = cell(10,1);
            obj.G_basis = cell(4,1);

            % Bases 1-3: principal moments of inertia.
            for k = 1:3
                E = zeros(3,3); E(k,k) = 1;
                Gi = zeros(6,6); Gi(1:3,1:3) = E;
                obj.I_basis{k} = Gi;
            end

            % Bases 4-6: cross (off-diagonal) inertias.
            pairs = [1 2; 2 3; 1 3];
            for idx = 1:3
                i = pairs(idx,1); j = pairs(idx,2);
                E = zeros(3,3); E(i,j) = 1; E(j,i) = 1;
                Gi = zeros(6,6); Gi(1:3,1:3) = E;
                obj.I_basis{3+idx} = Gi;
            end

            % Basis 7: mass (translational inertia).
            Gi = zeros(6,6); Gi(4:6,4:6) = eye(3);
            obj.I_basis{7} = Gi;

            % Bases 8-10: mass*CoG coupling (rotation-translation cross terms).
            for ax = 1:3
                e = zeros(3,1); e(ax) = 1;
                S = vt.se3.hat3(e);
                Gi = zeros(6,6);
                Gi(1:3,4:6) = S;
                Gi(4:6,1:3) = -S;
                obj.I_basis{7+ax} = Gi;
            end

            % Gravity bases: mass contribution.
            G7 = zeros(6,6); G7(4:6,4:6) = eye(3);
            obj.G_basis{1} = G7;
            % Gravity bases: CoG coupling to gravity torque.
            for ax = 1:3
                e = zeros(3,1); e(ax) = 1;
                S = vt.se3.hat3(e);
                Gi = zeros(6,6); Gi(1:3,1:3) = S;
                obj.G_basis{1+ax} = Gi;
            end
        end

        function [m_hat, cog_hat, Iparams_hat] = unpackTheta(~, theta)
            %UNPACKTHETA Convert parameter vector into physical values.
            %   Input:
            %     theta - 10x1 parameter vector.
            %   Outputs:
            %     m_hat - mass estimate.
            %     cog_hat - 3x1 CoG estimate.
            %     Iparams_hat - inertia parameter estimate.
            th = theta(:);
            Iparams_hat = [th(1), th(2), th(3), th(4), th(5), th(6)];
            m_hat = max(th(7), 1e-9);  % clamp to avoid division by zero in CoG recovery
            cog_hat = [th(8); th(9); th(10)] / m_hat;
        end

        function Y = regressor(obj, H_err, H, V, Vd, Ades, Ad_inv_err, V_e)
            %REGRESSOR Build the 6x10 dynamic regressor matrix Y.
            %   Y satisfies the identity (paper Eq. 18):
            %     Y*theta = ad_V^T * I6 * V + W_gravity - I6 * Ad^{-1}(H_e) * a_bar
            %   where a_bar = Ades + ad_Vd * Ad(H_e) * V_e.
            %   Each column corresponds to one element of the parameter vector.
            %
            %   The i-th column is:
            %     col_i = ad_V^T * B_i * V - B_i * Ad^{-1}(H_e) * a_bar
            %   plus, for parameters 7-10 (mass/CoG), the gravity contribution:
            %     col_i += G_basis{i-6} * [g_body; g_body]
            %
            %   Inputs:
            %     H_err - pose error H_d^{-1} * H.
            %     H - current pose.
            %     V, Vd - actual/desired body velocity.
            %     Ades - desired acceleration.
            %     Ad_inv_err - precomputed Ad_inv(H_err) (optional).
            %     V_e - precomputed velocity error (optional).
            %   Output:
            %     Y - 6x10 regressor matrix.
            if nargin < 7 || isempty(Ad_inv_err)
                Ad_inv_err = vt.se3.Ad_inv(H_err);
            end
            if nargin < 8 || isempty(V_e)
                V_e = V - Ad_inv_err * Vd;
            end
            a_bar = Ades + vt.se3.adV(Vd) * (vt.se3.Ad(H_err) * V_e);

            R = H(1:3,1:3);
            g_body = R' * [0;0;-obj.g];
            gvec = [g_body; g_body];

            cols = cell(10,1);
            for i = 1:10
                term = vt.se3.adV(V).' * obj.I_basis{i} * V ...
                       - obj.I_basis{i} * Ad_inv_err * a_bar;
                if i >= 7
                    term = term + obj.G_basis{i-6} * gvec;
                end
                cols{i} = term;
            end
            Y = [cols{:}];
        end
    end
end
