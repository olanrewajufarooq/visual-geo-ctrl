classdef TestLogger < matlab.unittest.TestCase
    %TESTLOGGER Tests for Logger preallocated buffering.

    methods (Test)
        function testAppendNSamplesFinalizesNRows(testCase)
            logger = fth.core.Logger();
            logger.reserve(100);
            for k = 1:100
                logger.append(k * 0.01, testCase.dummyActual(), ...
                    testCase.dummyDesired(), testCase.dummyCmd());
            end
            logs = logger.finalize();
            testCase.verifyEqual(size(logs.t, 1), 100);
            testCase.verifyEqual(size(logs.actual.pos, 1), 100);
        end

        function testAppendWithoutReserveWorks(testCase)
            % Logger must work even if reserve() is never called.
            logger = fth.core.Logger();
            for k = 1:10
                logger.append(k * 0.01, testCase.dummyActual(), ...
                    testCase.dummyDesired(), testCase.dummyCmd());
            end
            logs = logger.finalize();
            testCase.verifyEqual(size(logs.t, 1), 10);
        end

        function testAppendPastCapacityGrows(testCase)
            logger = fth.core.Logger();
            logger.reserve(5);
            for k = 1:20   % 4x the reserved capacity
                logger.append(k * 0.01, testCase.dummyActual(), ...
                    testCase.dummyDesired(), testCase.dummyCmd());
            end
            logs = logger.finalize();
            testCase.verifyEqual(size(logs.t, 1), 20);
        end

        function testFinalizeReturnsCorrectValues(testCase)
            logger = fth.core.Logger();
            logger.reserve(3);
            for k = 1:3
                act = testCase.dummyActual();
                act.pos = [k, k+1, k+2];
                logger.append(k * 1.0, act, testCase.dummyDesired(), testCase.dummyCmd());
            end
            logs = logger.finalize();
            testCase.verifyEqual(logs.t, [1; 2; 3], 'AbsTol', 1e-12);
            testCase.verifyEqual(logs.actual.pos(1,:), [1, 2, 3], 'AbsTol', 1e-12);
            testCase.verifyEqual(logs.actual.pos(3,:), [3, 4, 5], 'AbsTol', 1e-12);
        end

        function testExplicitTimingWrittenAndTruncated(testCase)
            % Verify that explicit timing structs are written correctly and
            % finalize() returns the right Nx1 shape (not zero-padded).
            logger = fth.core.Logger();
            logger.reserve(4);
            for k = 1:3
                timing.controlTime    = k * 0.01;
                timing.adaptationTime = k * 0.02;
                logger.append(k * 0.01, testCase.dummyActual(), ...
                    testCase.dummyDesired(), testCase.dummyCmd(), timing);
            end
            logs = logger.finalize();
            testCase.verifyEqual(size(logs.timing.controlTime), [3 1]);
            testCase.verifyEqual(logs.timing.controlTime, [0.01; 0.02; 0.03], 'AbsTol', 1e-14);
            testCase.verifyEqual(logs.timing.adaptationTime, [0.02; 0.04; 0.06], 'AbsTol', 1e-14);
        end

        function testReserveAfterAppendErrors(testCase)
            % reserve() after any append() must error to prevent silent data loss.
            logger = fth.core.Logger();
            logger.reserve(10);
            logger.append(0.01, testCase.dummyActual(), testCase.dummyDesired(), testCase.dummyCmd());
            testCase.verifyError(@() logger.reserve(20), 'fth:Logger:reserveAfterAppend');
        end
    end

    methods (Access = private)
        function s = dummyActual(~)
            s.pos    = zeros(1,3);
            s.rpy    = zeros(1,3);
            s.linVel = zeros(1,3);
            s.angVel = zeros(1,3);
        end

        function s = dummyDesired(~)
            s.pos    = zeros(1,3);
            s.rpy    = zeros(1,3);
            s.linVel = zeros(1,3);
            s.angVel = zeros(1,3);
            s.acc6   = zeros(1,6);
        end

        function s = dummyCmd(~)
            s.wrenchF = zeros(1,3);
            s.wrenchT = zeros(1,3);
        end
    end
end
