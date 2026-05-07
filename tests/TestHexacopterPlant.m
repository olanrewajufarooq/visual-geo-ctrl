classdef TestHexacopterPlant < matlab.unittest.TestCase
    %TESTHEXACOPTERPLANT Unit tests for plant state and ground defaults.

    methods (Test)
        function testConstructorReadsGroundDefaultsFromConfig(testCase)
            cfg = fth.core.Config();
            plant = fth.plant.HexacopterPlant(cfg);
            testCase.verifyTrue(plant.groundEnable);
            testCase.verifyEqual(plant.groundHeight, 0);
            testCase.verifyEqual(plant.groundStiffness, 5000);
            testCase.verifyEqual(plant.groundDamping, 200);
            testCase.verifyEqual(plant.groundFriction, 0.3);
        end

        function testResetSetsState(testCase)
            cfg = fth.core.Config();
            plant = fth.plant.HexacopterPlant(cfg);
            H0 = eye(4);
            H0(1:3,4) = [1; 2; 3];
            V0 = (1:6).';

            plant.reset(H0, V0);
            [H, V] = plant.getState();
            testCase.verifyEqual(H, H0);
            testCase.verifyEqual(V, V0);
        end

        function testStepWithZeroWrenchIsFinite(testCase)
            cfg = fth.core.Config();
            plant = fth.plant.HexacopterPlant(cfg);
            plant.reset(eye(4), zeros(6,1));
            plant.step(0.005, zeros(6,1));
            [H, V] = plant.getState();
            testCase.verifyTrue(all(isfinite(H), 'all'));
            testCase.verifyTrue(all(isfinite(V)));
        end

        function testStepUsesPositiveCoriolisSign(testCase)
            cfg = fth.core.Config();
            cfg.vehicle.g = 0;
            cfg.sim.groundEnable = false;
            plant = fth.plant.HexacopterPlant(cfg);

            H0 = eye(4);
            H0(3,4) = 5;
            V0 = [0.2; -0.3; 0.4; 0.5; -0.6; 0.7];
            dt = 1e-4;
            C = fth.se3.adV(V0)' * plant.I6 * V0;
            expectedV = V0 + dt * (plant.I6 \ C);

            plant.reset(H0, V0);
            plant.step(dt, zeros(6,1));
            [~, V] = plant.getState();

            testCase.verifyEqual(V, expectedV, 'AbsTol', 1e-12);
        end

        function testUpdateParametersChangesMassCogAndInertia(testCase)
            cfg = fth.core.Config();
            plant = fth.plant.HexacopterPlant(cfg);
            newMass = cfg.vehicle.m + 1.0;
            newCoG = [0.1; -0.05; 0.02];
            newI = cfg.vehicle.I_params(:) + 0.01;

            plant.updateParameters(newMass, newCoG, newI);

            testCase.verifyEqual(plant.m, newMass);
            testCase.verifyEqual(plant.CoG, newCoG);
            testCase.verifyEqual(size(plant.I6), [6 6]);
            testCase.verifyTrue(all(isfinite(plant.I6), 'all'));
        end

        function testDropPayloadConservesLinearMomentum(testCase)
            cfg = fth.core.Config();
            cfg.vehicle.g = 0;
            cfg.sim.groundEnable = false;
            plant = fth.plant.HexacopterPlant(cfg);

            m_base = cfg.vehicle.m;
            I_base = cfg.vehicle.I_params;
            cog_base = cfg.vehicle.CoG(:);

            m_payload = 0.5;
            cog_payload = [0; 0; -0.1];
            [m_comp, I_comp, cog_comp] = fth.utils.addPayload(m_base, I_base, cog_base, m_payload, cog_payload);
            plant.updateParameters(m_comp, cog_comp, I_comp);

            H0 = eye(4); H0(3,4) = 5;
            V0 = [0; 0; 0; 1.0; 0; 0];  % pure x-translation
            plant.reset(H0, V0);

            I6_before = plant.I6;
            p_before = I6_before * V0;

            plant.dropPayload(m_base, cog_base, I_base);

            [~, V_after] = plant.getState();
            I6_after = plant.I6;
            p_after = I6_after * V_after;

            testCase.verifyEqual(p_after, p_before, 'AbsTol', 1e-12);
        end

        function testDropPayloadConservesGeneralizedMomentum(testCase)
            cfg = fth.core.Config();
            cfg.vehicle.g = 0;
            cfg.sim.groundEnable = false;
            plant = fth.plant.HexacopterPlant(cfg);

            m_base = cfg.vehicle.m;
            I_base = cfg.vehicle.I_params;
            cog_base = cfg.vehicle.CoG(:);

            m_payload = 0.5;
            cog_payload = [0.05; -0.03; -0.1];
            [m_comp, I_comp, cog_comp] = fth.utils.addPayload(m_base, I_base, cog_base, m_payload, cog_payload);
            plant.updateParameters(m_comp, cog_comp, I_comp);

            H0 = eye(4); H0(3,4) = 5;
            V0 = [0.1; -0.2; 0.15; 0.8; -0.5; 0.3];
            plant.reset(H0, V0);

            I6_before = plant.I6;
            p_before = I6_before * V0;

            plant.dropPayload(m_base, cog_base, I_base);

            [~, V_after] = plant.getState();
            I6_after = plant.I6;
            p_after = I6_after * V_after;

            testCase.verifyEqual(p_after, p_before, 'AbsTol', 1e-12);
        end

        function testDropPayloadNoOpWhenMassUnchanged(testCase)
            cfg = fth.core.Config();
            cfg.vehicle.g = 0;
            cfg.sim.groundEnable = false;
            plant = fth.plant.HexacopterPlant(cfg);

            H0 = eye(4); H0(3,4) = 5;
            V0 = [0.1; -0.2; 0.15; 0.8; -0.5; 0.3];
            plant.reset(H0, V0);

            m_same = cfg.vehicle.m;
            I_same = cfg.vehicle.I_params;
            cog_same = cfg.vehicle.CoG(:);

            plant.dropPayload(m_same, cog_same, I_same);

            [~, V_after] = plant.getState();
            testCase.verifyEqual(V_after, V0, 'AbsTol', 1e-12);
        end

        function testUpdateParametersMassOnlyRecomputesI6(testCase)
            % Verify I6 is recomputed even when only mass changes (no Iparams arg).
            cfg = fth.core.Config();
            plant = fth.plant.HexacopterPlant(cfg);
            I6_before = plant.I6;
            plant.updateParameters(plant.m * 2, []);  % double the mass, no Iparams
            testCase.verifyNotEqual(plant.I6, I6_before);
            % Also verify mass actually changed.
            testCase.verifyEqual(plant.m, cfg.vehicle.m * 2, 'AbsTol', 1e-12);
        end

        function testUpdateParametersCoGOnlyRecomputesI6(testCase)
            % Verify I6 is recomputed even when only CoG changes (no Iparams arg).
            cfg = fth.core.Config();
            plant = fth.plant.HexacopterPlant(cfg);
            I6_before = plant.I6;
            % new_cog is deliberately non-zero to ensure the off-diagonal
            % coupling terms m*hat3(CoG) in I6 change; default CoG is also
            % non-zero, so any different value guarantees I6 changes.
            new_cog = [0.01; 0.02; -0.03];
            plant.updateParameters([], new_cog);  % CoG only, no Iparams
            testCase.verifyNotEqual(plant.I6, I6_before);
            testCase.verifyEqual(plant.CoG, new_cog, 'AbsTol', 1e-12);
        end
    end
end
