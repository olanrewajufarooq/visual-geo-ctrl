classdef NoAdaptation < fth.ctrl.adapt.AdaptationBase
    %NOADAPTATION Fixed-parameter model (no adaptation).
    %   Returns nominal mass, CoG, and inertia parameters without updates.
    %
    %   Use this mode for baseline (non-adaptive) simulations.
    properties (Access = private)
        m
        CoG
        Iparams
        I6
    end

    methods
        function obj = NoAdaptation(cfg)
            %NOADAPTATION Cache nominal parameters from config.
            %   Input:
            %     cfg - configuration with vehicle fields.
            obj.m      = cfg.vehicle.m;
            obj.CoG    = cfg.vehicle.CoG(:);
            obj.Iparams = cfg.vehicle.I_params(:);
            if isfield(cfg.vehicle, 'I6')
                obj.I6 = cfg.vehicle.I6;
            else
                obj.I6 = fth.se3.getGeneralizedInertia(obj.m, obj.Iparams, obj.CoG);
            end
        end

        function params = update(obj, ~, ~, ~, ~, ~, ~, ~, ~, ~)
            %UPDATE Return nominal parameters without changes.
            %   (All inputs ignored.)
            %   Output:
            %     params - struct with m, CoG, Iparams, I6.
            params = obj.getParams();
        end

        function params = getParams(obj)
            %GETPARAMS Return fixed parameters and inertia matrix.
            params = struct('m', obj.m, 'CoG', obj.CoG, 'Iparams', obj.Iparams, 'I6', obj.I6);
        end

        function [m_hat, cog_hat, Iparams_hat] = getEstimate(~)
            %GETESTIMATE No estimates available for fixed-parameter mode.
            m_hat = [];  cog_hat = [];  Iparams_hat = [];
        end

        function diagnostics = getDiagnostics(~)
            %GETDIAGNOSTICS No adaptation diagnostics in fixed mode.
            diagnostics = struct('infoMatrix', [], 'updateCount', 0);
        end

        function setEstimatePi(obj, pi)
            %SETESTIMATEPI Seed fixed parameters from a pi vector.
            % Input:
            % pi - 10×1 vector [m; hx; hy; hz; Ixx; Iyy; Izz; Ixy; Ixz; Iyz].
            [obj.m, obj.CoG, obj.Iparams] = obj.unpackPi(pi(:));
            obj.I6 = fth.ctrl.adapt.AdaptationUtils.params2genInertia(pi(:));
        end
    end
end
