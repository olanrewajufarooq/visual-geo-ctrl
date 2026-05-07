classdef TestUtils < matlab.unittest.TestCase
    %TESTUTILS Tests for rotation and inertia utility functions.

    methods (Test)
        function testRpyRoundTripRandom(testCase)
            % Round-trip rpy2rotm -> rotm2rpy should recover original angles.
            rng(42);
            for k = 1:50
                rpy_in = (rand(3,1) - 0.5) * pi;
                rpy_in(2) = rpy_in(2) * 0.8;  % keep away from gimbal lock
                R = fth.utils.rpy2rotm(rpy_in);
                rpy_out = fth.utils.rotm2rpy(R);
                testCase.verifyEqual(rpy_out, rpy_in, 'AbsTol', 1e-10, ...
                    sprintf('Round-trip failed at iteration %d', k));
            end
        end

        function testRotm2RpyGimbalLockWarns(testCase)
            % Pitch = pi/2 is gimbal lock; expect a warning.
            rpy_in = [0.3; pi/2; 0.5];
            R = fth.utils.rpy2rotm(rpy_in);
            testCase.verifyWarning(@() fth.utils.rotm2rpy(R), 'fth:gimbalLock');
        end

        function testRpy2RotmOutputSize(testCase)
            rpy = [0.1; 0.2; 0.3];
            R = fth.utils.rpy2rotm(rpy);
            testCase.verifyEqual(size(R), [3 3]);
        end

        function testRpyRoundTripZero(testCase)
            rpy_in = [0; 0; 0];
            R = fth.utils.rpy2rotm(rpy_in);
            testCase.verifyEqual(R, eye(3), 'AbsTol', 1e-14);
            rpy_out = fth.utils.rotm2rpy(R);
            testCase.verifyEqual(rpy_out, rpy_in, 'AbsTol', 1e-14);
        end

        function testInertiaFromParamsSymmetric(testCase)
            Ip = [0.01, 0.02, 0.03, 0.001, 0.002, 0.003];
            J = fth.utils.inertiaFromParams(Ip);
            testCase.verifyEqual(J, J', 'AbsTol', 1e-14);
        end

        function testAddPayloadConservesMass(testCase)
            m_base = 2.0;
            Ip = [0.01, 0.02, 0.03, 0, 0, 0];
            cog = [0; 0; 0];
            m_payload = 0.5;
            cog_p = [0; 0; -0.1];
            [m_total, ~, ~] = fth.utils.addPayload(m_base, Ip, cog, m_payload, cog_p);
            testCase.verifyEqual(m_total, 2.5, 'AbsTol', 1e-12);
        end

        function testAddPayloadRoundTripOffDiagonal(testCase)
            % Round-trip: addPayload -> inertiaFromParams must preserve
            % off-diagonal terms. Catches Iyz/Ixz packing order bugs.
            m_base = 2.0;
            % Non-diagonal Iparams: [Ixx Iyy Izz Ixy Iyz Ixz]
            Ip = [0.10, 0.20, 0.30, 0.01, 0.02, 0.03];
            cog = [0; 0; 0];
            % Zero payload so combined inertia equals base inertia exactly.
            [~, Ip_out, ~] = fth.utils.addPayload(m_base, Ip, cog, 0, [0;0;0]);
            J_in  = fth.utils.inertiaFromParams(Ip);
            J_out = fth.utils.inertiaFromParams(Ip_out);
            testCase.verifyEqual(J_out, J_in, 'AbsTol', 1e-12, ...
                'Off-diagonal inertia terms corrupted by addPayload packing');
        end
    end
end
