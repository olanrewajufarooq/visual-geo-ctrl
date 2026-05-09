classdef PotentialFactory
    %POTENTIALFACTORY Create potential function from config.
    %   Reads cfg.controller.potential to select the implementation.
    %
    %   Valid potType strings:
    %     'log'          - SE(3) log-map potential (logPotential)
    %     'inertia-gain' - gains in the inertia frame (inertiaGainPotential)
    %     'body-gain'    - gains in the body frame   (bodyGainPotential)
    %     'ref-gain'     - gains in the reference frame (refGainPotential)
    %     'sym-inv'      - symmetric and invariant    (symInvPotential)
    %
    %   Config fields: controller.potential, controller.Kp (6x1).
    %   K_R  = diag(Kp(1:3)),  K_xi = diag(Kp(4:6)).
    methods (Static)
        function potential = create(cfg)
            %CREATE Instantiate the configured potential function.
            %   Input:
            %     cfg - configuration with controller.potential and Kp.
            %   Output:
            %     potential - PotentialBase implementation.
            potType = cfg.controller.potential;
            K_R  = diag(cfg.controller.Kp(1:3));
            K_xi = diag(cfg.controller.Kp(4:6));
            Kp   = blkdiag(K_R, K_xi);

            switch lower(potType)
                case 'log'
                    potential = fth.ctrl.potential.logPotential(Kp);
                case 'inertia-gain'
                    potential = fth.ctrl.potential.inertiaGainPotential(K_R, K_xi);
                case 'body-gain'
                    potential = fth.ctrl.potential.bodyGainPotential(K_R, K_xi);
                case 'ref-gain'
                    potential = fth.ctrl.potential.refGainPotential(K_R, K_xi);
                case 'sym-inv'
                    potential = fth.ctrl.potential.symInvPotential(K_R, K_xi);
                otherwise
                    error('fth:PotentialFactory:UnknownType', ...
                        ['Unknown potential type: ''%s''. ', ...
                         'Valid types: log, inertia-gain, body-gain, ref-gain, sym-inv.'], ...
                        potType);
            end
        end
    end
end
