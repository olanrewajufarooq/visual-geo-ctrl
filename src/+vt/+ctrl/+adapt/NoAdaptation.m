classdef NoAdaptation < vt.ctrl.adapt.AdaptationBase
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
            %   Output:
            %     obj - NoAdaptation instance.
            obj.m = cfg.vehicle.m;
            obj.CoG = cfg.vehicle.CoG(:);
            obj.Iparams = cfg.vehicle.I_params(:);
            if isfield(cfg.vehicle, 'I6')
                obj.I6 = cfg.vehicle.I6;
            else
                obj.I6 = vt.utils.getGeneralizedInertia(obj.m, obj.Iparams, obj.CoG);
            end
        end

        function params = update(obj, ~, ~, ~, ~, ~, ~)
            %UPDATE Return nominal parameters without changes.
            %   Output:
            %     params - struct with m, CoG, Iparams, I6.
            params = obj.getParams();
        end

        function params = getParams(obj)
            %GETPARAMS Return fixed parameters and inertia matrix.
            %   Output:
            %     params - struct with m, CoG, Iparams, I6.
            params = struct('m', obj.m, 'CoG', obj.CoG, 'Iparams', obj.Iparams, 'I6', obj.I6);
        end

        function [m_hat, cog_hat, Iparams_hat] = getEstimate(obj)
            %GETESTIMATE No estimates available for fixed-parameter mode.
            %   Outputs are empty arrays.
            m_hat = [];
            cog_hat = [];
            Iparams_hat = [];
        end

        function diagnostics = getDiagnostics(~)
            %GETDIAGNOSTICS No adaptation diagnostics in fixed mode.
            diagnostics = struct('infoMatrix', [], 'updateCount', 0);
        end

        function setEstimateTheta(~, ~)
            %SETESTIMATETHETA No-op for fixed-parameter mode.
        end
    end
end
