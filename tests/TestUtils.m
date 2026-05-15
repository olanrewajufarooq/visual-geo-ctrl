classdef TestUtils < matlab.unittest.TestCase
    %TESTUTILS Tests for rotation and inertia utility functions.

    methods (Test)
        function testRpyRoundTripRandom(testCase)
            % Round-trip rpy2rotm -> rotm2rpy should recover original angles.
            rng(42);
            for k = 1:50
                rpy_in = (rand(3,1) - 0.5) * pi;
                rpy_in(2) = rpy_in(2) * 0.8;  % keep away from gimbal lock
                R = fth.se3.rpy2rotm(rpy_in);
                rpy_out = fth.se3.rotm2rpy(R);
                testCase.verifyEqual(rpy_out, rpy_in, 'AbsTol', 1e-10, ...
                    sprintf('Round-trip failed at iteration %d', k));
            end
        end

        function testRotm2RpyGimbalLockWarns(testCase)
            % Pitch = pi/2 is gimbal lock; expect a warning.
            rpy_in = [0.3; pi/2; 0.5];
            R = fth.se3.rpy2rotm(rpy_in);
            testCase.verifyWarning(@() fth.se3.rotm2rpy(R), 'fth:gimbalLock');
        end

        function testRpy2RotmOutputSize(testCase)
            rpy = [0.1; 0.2; 0.3];
            R = fth.se3.rpy2rotm(rpy);
            testCase.verifyEqual(size(R), [3 3]);
        end

        function testRpyRoundTripZero(testCase)
            rpy_in = [0; 0; 0];
            R = fth.se3.rpy2rotm(rpy_in);
            testCase.verifyEqual(R, eye(3), 'AbsTol', 1e-14);
            rpy_out = fth.se3.rotm2rpy(R);
            testCase.verifyEqual(rpy_out, rpy_in, 'AbsTol', 1e-14);
        end

        function testRotInertiaVec2MatSymmetric(testCase)
            Ip = [0.01, 0.02, 0.03, 0.001, 0.002, 0.003];
            J = fth.se3.rotInertiaVec2Mat(Ip);
            testCase.verifyEqual(J, J', 'AbsTol', 1e-14);
        end

    end
end
