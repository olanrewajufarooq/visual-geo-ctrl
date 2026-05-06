classdef AdaptationFactory
    %ADAPTATIONFACTORY Create adaptation implementations from config.
    %   Uses cfg.controller.adaptation to choose the strategy.
    %
    %   Supported modes: none, euclidean, geo-aware, geo-enforced.
    methods (Static)
        function adaptation = create(cfg)
            %CREATE Instantiate the configured adaptation strategy.
            %   Input:
            %     cfg - configuration with controller.adaptation field.
            %   Output:
            %     adaptation - AdaptationBase implementation.
            adaptType = 'none';
            if isfield(cfg.controller, 'adaptation')
                adaptType = cfg.controller.adaptation;
            end

            if strcmpi(adaptType, 'none')
                adaptation = fth.ctrl.adapt.NoAdaptation(cfg);
                return;
            end

            switch lower(adaptType)
                case 'euclidean'
                    adaptation = fth.ctrl.adapt.EuclideanAdaptation(cfg);
                case 'geo-aware'
                    adaptation = fth.ctrl.adapt.GeoAwareAdaptation(cfg);
                otherwise
                    warning('Adaptation type %s not implemented; using euclidean.', adaptType);
                    adaptation = fth.ctrl.adapt.EuclideanAdaptation(cfg);
            end
        end
    end
end
