classdef AdaptationFactory
    %ADAPTATIONFACTORY Create adaptation instances from config.
    methods (Static)
        function adaptation = create(cfg, coriolisFactor)
            %CREATE Instantiate the configured adaptation law.
            %   Input:
            %     cfg - fth.sim.Config instance.
            %     coriolisFactor - optional CoriolisFactorBase instance shared
            %                      with the controller.
            %
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

            % Gamma=0 is the batch sentinel meaning "disabled for this run".
            gammaIsZero = isfield(cfg.controller, 'Gamma') && ...
                isscalar(cfg.controller.Gamma) && cfg.controller.Gamma == 0;

            switch lower(adaptType)
                case 'none'
                    adaptation = fth.ctrl.adapt.NoAdaptation(cfg);
                case 'euclidean'
                    if gammaIsZero
                        adaptation = fth.ctrl.adapt.NoAdaptation(cfg);
                    else
                        adaptation = fth.ctrl.adapt.EuclideanAdaptation(cfg);
                    end
                case 'bregman'
                    if gammaIsZero
                        adaptation = fth.ctrl.adapt.NoAdaptation(cfg);
                    else
                        adaptation = fth.ctrl.adapt.BregmanDivAdaptation(cfg);
                    end
                otherwise
                    error('fth:AdaptationFactory:UnknownMode', ...
                        'Unrecognised adaptation mode: ''%s''.', adaptType);
            end
            
            if nargin >= 2 && ~isempty(coriolisFactor)
                adaptation.setCoriolisFactor(coriolisFactor);
            end
        end
    end
end
