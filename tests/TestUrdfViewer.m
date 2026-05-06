classdef TestUrdfViewer < matlab.unittest.TestCase
    %TESTURDFVIEWER Tests for URDF sanitization logic.

    properties (TestParameter)
    end

    properties (Access = private)
        urdfPath
    end

    methods (TestMethodSetup)
        function resolveUrdfPath(testCase)
            testCase.urdfPath = fullfile(fileparts(fileparts(mfilename('fullpath'))), ...
                'assets', 'hexacopter_description', 'urdf', 'variable_tilt_hexacopter.urdf');
        end
    end

    methods (Test)
        function testSanitizeRemovesSensorTags(testCase)
            sanitized = fth.plot.UrdfViewer.sanitizeUrdf(testCase.urdfPath);
            txt = fileread(sanitized);
            testCase.verifyEmpty(regexp(txt, '<sensor\b'));
        end

        function testSanitizeRemovesPluginTags(testCase)
            sanitized = fth.plot.UrdfViewer.sanitizeUrdf(testCase.urdfPath);
            txt = fileread(sanitized);
            testCase.verifyEmpty(regexp(txt, '<plugin\b'));
        end

        function testSanitizeRemovesPoseTags(testCase)
            sanitized = fth.plot.UrdfViewer.sanitizeUrdf(testCase.urdfPath);
            txt = fileread(sanitized);
            testCase.verifyEmpty(regexp(txt, '<pose\b'));
        end

        function testSanitizeRemovesGravityAndVelocityDecay(testCase)
            sanitized = fth.plot.UrdfViewer.sanitizeUrdf(testCase.urdfPath);
            txt = fileread(sanitized);
            testCase.verifyEmpty(regexp(txt, '<gravity\b'));
            testCase.verifyEmpty(regexp(txt, '<velocity_decay'));
        end

        function testSanitizedUrdfIsValidXml(testCase)
            sanitized = fth.plot.UrdfViewer.sanitizeUrdf(testCase.urdfPath);
            try
                xmlread(sanitized);
                isValid = true;
            catch
                isValid = false;
            end
            testCase.verifyTrue(isValid);
        end

        function testSanitizedUrdfImportsWithRoboticsToolbox(testCase)
            if exist('importrobot', 'file') ~= 2
                testCase.assumeFail('Robotics System Toolbox not available.');
            end
            sanitized = fth.plot.UrdfViewer.sanitizeUrdf(testCase.urdfPath);
            robot = importrobot(sanitized, 'DataFormat', 'column');
            testCase.verifyNotEmpty(robot);
            testCase.verifyGreaterThan(robot.NumBodies, 0);
        end
    end
end
