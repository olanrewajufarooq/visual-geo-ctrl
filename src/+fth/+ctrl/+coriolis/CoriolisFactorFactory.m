classdef CoriolisFactorFactory
    %CORIOLISFACTORFACTORY Instantiate Coriolis factorization from config.
    %   Config field: controller.coriolisFactorization (default: 'basic').
    methods (Static)
        function coriolisFactor = create(cfg)
            %CREATE Return a CoriolisFactorBase implementation.
            %   Input:
            %     cfg - configuration with controller.coriolisFactorization.
            %   Output:
            %     coriolisFactor - CoriolisFactorBase instance.
            if isfield(cfg.controller, 'coriolisFactorization') && ...
                    ~isempty(cfg.controller.coriolisFactorization)
                form = lower(cfg.controller.coriolisFactorization);
            else
                form = 'basic';
            end

            switch form
                case 'basic'
                    coriolisFactor = fth.ctrl.coriolis.basicCoriolisFactor();
                case 'consistent'
                    coriolisFactor = fth.ctrl.coriolis.consistentCoriolisFactor();
                otherwise
                    error('fth:CoriolisFactorFactory:UnknownForm', ...
                        'Unknown coriolis factorization: ''%s''.', form);
            end
        end
    end
end
