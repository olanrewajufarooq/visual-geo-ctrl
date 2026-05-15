classdef ControllerFactory
    %CONTROLLERFACTORY Assemble a ControllerWrench from config.
    methods (Static)
        function ctrl = create(cfg)
            %CREATE Build a ControllerWrench from configuration.
            %   Input:
            %     cfg - fth.sim.Config instance.
            %   Output:
            %     ctrl - fth.ctrl.ControllerWrench instance.
            coriolisFactor = fth.ctrl.coriolis.CoriolisFactorFactory.create(cfg);
            adaptation     = fth.ctrl.adapt.AdaptationFactory.create(cfg, coriolisFactor);
            Kd             = diag(cfg.controller.Kd(:));
            ctrl           = fth.ctrl.ControllerWrench(adaptation, Kd);
        end
    end
end
