classdef PotentialFactory
    %POTENTIALFACTORY Create potential models from config.
    %   Selects between Lie-algebra and separated attitude/position models.
    %
    %   Config fields: controller.potential, controller.Kp.
    methods (Static)
        function potential = create(cfg)
            %CREATE Instantiate the configured potential function.
            %   Input:
            %     cfg - configuration with controller.potential and Kp.
            %   Output:
            %     potential - PotentialBase implementation.
            potType = cfg.controller.potential;
            Kp = diag(cfg.controller.Kp);
            Kp_att = diag(cfg.controller.Kp(1:3));
            Kp_pos = diag(cfg.controller.Kp(4:6));

            if strcmpi(potType, 'liealgebra')
                potential = vt.ctrl.potential.LieAlgebraPotential(Kp);
            else
                potential = vt.ctrl.potential.SeparatePotential(Kp_att, Kp_pos);
            end
        end
    end
end
