classdef AdaptationFactory
    %ADAPTATIONFACTORY Create adaptation instances from config.
    methods (Static)
        function adaptation = create(cfg)
            %CREATE Instantiate the configured adaptation law.
            %   Input:
            %     cfg - fth.sim.Config instance.
            %   Output:
            %     adaptation - AdaptationBase subclass instance.
            %
            %   Supported cfg.controller.adaptation values:
            %     'none'      - NoAdaptation (default)
            %     'euclidean' - EuclideanAdaptation
            %     'bregman'   - BregmanDivAdaptation
            adaptType = 'none';
            if isfield(cfg.controller, 'adaptation')
                adaptType = cfg.controller.adaptation;
            end

            switch lower(adaptType)
                case 'none'
                    adaptation = fth.ctrl.adapt.NoAdaptation(cfg);
                case 'euclidean'
                    adaptation = fth.ctrl.adapt.EuclideanAdaptation(cfg);
                case 'bregman'
                    adaptation = fth.ctrl.adapt.BregmanDivAdaptation(cfg);
                otherwise
                    error('fth:AdaptationFactory:UnknownMode', ...
                        'Unrecognised adaptation mode: ''%s''.', adaptType);
            end
        end
    end
end
