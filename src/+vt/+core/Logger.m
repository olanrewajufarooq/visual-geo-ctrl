classdef Logger < handle
    %LOGGER Collects simulation signals into structured logs.
    %   Stores time vectors and per-step structs for actual, desired, and
    %   command signals, plus control/adaptation timing data.
    %
    %   Output structure:
    %     logs.t, logs.actual, logs.des, logs.cmd, logs.timing.
    properties (Access = private)
        t
        actual
        des
        cmd
        timing
    end

    methods
        function obj = Logger()
            %LOGGER Initialize empty log buffers.
            %   Output:
            %     obj - Logger instance with empty arrays.
            obj.t = [];
            obj.actual = obj.emptyActual();
            obj.des = obj.emptyDesired();
            obj.cmd = obj.emptyCmd();
            obj.timing = obj.emptyTiming();
        end

        function append(obj, t, actual, desired, cmd, timing)
            %APPEND Add a new sample to the log buffers.
            %   Inputs:
            %     t - scalar timestamp.
            %     actual - struct with pos/rpy/linVel/angVel fields.
            %     desired - struct with pos/rpy/linVel/angVel/acc6 fields.
            %     cmd - struct with wrenchF/wrenchT fields.
            %     timing - struct with controlTime/adaptationTime (optional).
            if nargin < 6 || isempty(timing)
                timing = obj.defaultTiming(t);
            end
            obj.t = [obj.t; t];
            obj.actual = obj.mergeStruct(obj.actual, actual);
            obj.des = obj.mergeStruct(obj.des, desired);
            obj.cmd = obj.mergeStruct(obj.cmd, cmd);
            obj.timing = obj.mergeStruct(obj.timing, timing);
        end

        function logs = finalize(obj)
            %FINALIZE Return logs as a struct of time-series arrays.
            %   Output:
            %     logs - struct containing t, actual, des, cmd, timing.
            logs.t = obj.t;
            logs.actual = obj.actual;
            logs.des = obj.des;
            logs.cmd = obj.cmd;
            logs.timing = obj.timing;
        end
    end

    methods (Access = private)
        function s = mergeStruct(~, s, new)
            %MERGESTRUCT Append fields from new sample struct.
            %   Inputs:
            %     s - existing struct of arrays.
            %     new - struct of sample values.
            %
            %   Output:
            %     s - updated struct with concatenated fields.
            fields = fieldnames(new);
            for k = 1:numel(fields)
                f = fields{k};
                s.(f) = [s.(f); new.(f)];
            end
        end

        function s = emptyActual(~)
            %EMPTYACTUAL Create empty actual-state buffers.
            %   Output fields: pos, rpy, linVel, angVel.
            s.pos = zeros(0,3);
            s.rpy = zeros(0,3);
            s.linVel = zeros(0,3);
            s.angVel = zeros(0,3);
        end

        function s = emptyDesired(~)
            %EMPTYDESIRED Create empty desired-state buffers.
            %   Output fields: pos, rpy, linVel, angVel, acc6.
            s.pos = zeros(0,3);
            s.rpy = zeros(0,3);
            s.linVel = zeros(0,3);
            s.angVel = zeros(0,3);
            s.acc6 = zeros(0,6);
        end

        function s = emptyCmd(~)
            %EMPTYCMD Create empty command buffers.
            %   Output fields: wrenchF, wrenchT.
            s.wrenchF = zeros(0,3);
            s.wrenchT = zeros(0,3);
        end

        function s = emptyTiming(~)
            %EMPTYTIMING Create empty timing buffers.
            %   Output fields: controlTime, adaptationTime.
            s.controlTime = zeros(0,1);
            s.adaptationTime = zeros(0,1);
        end

        function s = defaultTiming(~, t)
            %DEFAULTTIMING Populate timing defaults from t.
            %   Sets both control and adaptation time to t.
            s.controlTime = t;
            s.adaptationTime = t;
        end
    end
end
