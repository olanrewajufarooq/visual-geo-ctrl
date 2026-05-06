classdef ConsoleCapture < handle
    %CONSOLECAPTURE Manage command-window capture to file.
    %   Wraps MATLAB's diary and evalc for console output interception.
    %
    %   Usage:
    %     cc = vt.sim.ConsoleCapture();
    %     cc.beginDiary(resultsDir);
    %     ... simulation code ...
    %     cc.endDiary();
    %
    %     output = cc.capture(@() someFunction());
    properties (Access = private)
        logPath
        active
        callback
    end

    methods
        function obj = ConsoleCapture()
            %CONSOLECAPTURE Create an idle capture instance.
            obj.logPath = '';
            obj.active = false;
            obj.callback = [];
        end

        function beginDiary(obj, resultsDir)
            %BEGINDIARY Start writing command-window output to file.
            %   Input: resultsDir - directory for command_window.txt.
            if obj.active
                return;
            end
            obj.logPath = fullfile(resultsDir, 'command_window.txt');
            diary(obj.logPath);
            obj.active = true;
        end

        function endDiary(obj)
            %ENDDIARY Stop command-window capture when active.
            if ~obj.active
                return;
            end
            diary off;
            obj.active = false;
        end

        function tf = isActive(obj)
            %ISACTIVE Return true when diary capture is running.
            tf = obj.active;
        end

        function output = capture(obj, fn)
            %CAPTURE Capture command-window output from a callback using evalc.
            %   Input:  fn - function handle to execute.
            %   Output: output - captured console text.
            obj.callback = fn;
            cleanup = onCleanup(@() obj.clearCallback());
            output = evalc('obj.invokeCallback();');
            fprintf('%s', output);
            clear cleanup
        end
    end

    methods (Access = private)
        function invokeCallback(obj)
            %INVOKECALLBACK Execute the stored callback.
            if isempty(obj.callback)
                return;
            end
            obj.callback();
        end

        function clearCallback(obj)
            %CLEARCALLBACK Reset the stored callback.
            obj.callback = [];
        end
    end
end
