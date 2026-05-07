classdef Logger < handle
    %LOGGER Collects simulation signals into structured logs.
    %   Stores time vectors and per-step structs for actual, desired, and
    %   command signals, plus control/adaptation timing data.
    %
    %   Use reserve(n) before the simulation loop to preallocate buffers
    %   for n samples, enabling O(1) amortized append. Without reserve(),
    %   the buffer grows automatically via doubling.
    %
    %   Output structure:
    %     logs.t, logs.actual, logs.des, logs.cmd, logs.timing.
    properties (Access = private)
        t
        actual
        des
        cmd
        timing
        capacity   % current buffer capacity (rows allocated)
        count      % number of samples written so far
    end

    methods
        function obj = Logger()
            %LOGGER Initialize empty log buffers.
            %   Output:
            %     obj - Logger instance with empty arrays.
            obj.capacity = 0;
            obj.count = 0;
            obj.t = zeros(0, 1);
            obj.actual = obj.emptyActual();
            obj.des = obj.emptyDesired();
            obj.cmd = obj.emptyCmd();
            obj.timing = obj.emptyTiming();
        end

        function reserve(obj, n)
            %RESERVE Preallocate buffers for n samples.
            %   Call once before the simulation loop with the expected sample count.
            %   Errors if called after any append() to prevent silent data loss.
            %   Input:
            %     n - number of samples to preallocate.
            if obj.count > 0
                error('fth:Logger:reserveAfterAppend', ...
                    'reserve() must be called before any append().');
            end
            obj.capacity = n;
            obj.count = 0;
            obj.t = zeros(n, 1);
            obj.actual = obj.preallocActual(n);
            obj.des = obj.preallocDesired(n);
            obj.cmd = obj.preallocCmd(n);
            obj.timing = obj.preallocTiming(n);
        end

        function append(obj, t, actual, desired, cmd, timing)
            %APPEND Add a new sample to the log buffers (O(1) amortized).
            %   Inputs:
            %     t - scalar timestamp.
            %     actual - struct with pos/rpy/linVel/angVel fields.
            %     desired - struct with pos/rpy/linVel/angVel/acc6 fields.
            %     cmd - struct with wrenchF/wrenchT fields.
            %     timing - struct with controlTime/adaptationTime (optional).
            if nargin < 6 || isempty(timing)
                timing = obj.defaultTiming(t);
            end
            obj.count = obj.count + 1;
            if obj.count > obj.capacity
                obj.grow();
            end
            k = obj.count;
            obj.t(k) = t;
            obj.actual = obj.writeStruct(obj.actual, actual, k);
            obj.des = obj.writeStruct(obj.des, desired, k);
            obj.cmd = obj.writeStruct(obj.cmd, cmd, k);
            obj.timing = obj.writeStruct(obj.timing, timing, k);
        end

        function logs = finalize(obj)
            %FINALIZE Return logs truncated to the number of samples appended.
            %   Output:
            %     logs - struct containing t, actual, des, cmd, timing.
            n = obj.count;
            logs.t = obj.t(1:n);
            logs.actual = obj.truncStruct(obj.actual, n);
            logs.des = obj.truncStruct(obj.des, n);
            logs.cmd = obj.truncStruct(obj.cmd, n);
            logs.timing = obj.truncStruct(obj.timing, n);
        end
    end

    methods (Access = private)
        function s = writeStruct(~, s, new, k)
            %WRITESTRUCT Write scalar new sample into row k of preallocated struct s.
            fields = fieldnames(new);
            for i = 1:numel(fields)
                f = fields{i};
                s.(f)(k, :) = new.(f);
            end
        end

        function s = truncStruct(~, s, n)
            %TRUNCSTRUCT Truncate all fields of s to first n rows.
            fields = fieldnames(s);
            for i = 1:numel(fields)
                f = fields{i};
                s.(f) = s.(f)(1:n, :);
            end
        end

        function grow(obj)
            %GROW Double buffer capacity when full.
            newCap = max(obj.capacity * 2, 16);
            obj.t = [obj.t; zeros(newCap - obj.capacity, 1)];
            obj.actual = obj.growStruct(obj.actual, obj.capacity, newCap);
            obj.des = obj.growStruct(obj.des, obj.capacity, newCap);
            obj.cmd = obj.growStruct(obj.cmd, obj.capacity, newCap);
            obj.timing = obj.growStruct(obj.timing, obj.capacity, newCap);
            obj.capacity = newCap;
        end

        function s = growStruct(~, s, oldCap, newCap)
            %GROWSTRUCT Extend each field from oldCap to newCap rows.
            fields = fieldnames(s);
            for i = 1:numel(fields)
                f = fields{i};
                cols = size(s.(f), 2);
                s.(f) = [s.(f); zeros(newCap - oldCap, cols)];
            end
        end

        function s = preallocActual(~, n)
            s.pos = zeros(n,3);
            s.rpy = zeros(n,3);
            s.linVel = zeros(n,3);
            s.angVel = zeros(n,3);
        end

        function s = preallocDesired(~, n)
            s.pos = zeros(n,3);
            s.rpy = zeros(n,3);
            s.linVel = zeros(n,3);
            s.angVel = zeros(n,3);
            s.acc6 = zeros(n,6);
        end

        function s = preallocCmd(~, n)
            s.wrenchF = zeros(n,3);
            s.wrenchT = zeros(n,3);
        end

        function s = preallocTiming(~, n)
            s.controlTime = zeros(n,1);
            s.adaptationTime = zeros(n,1);
        end

        function s = defaultTiming(~, t)
            %DEFAULTTIMING Populate timing defaults from t.
            %   Sets both control and adaptation time to t.
            s.controlTime = t;
            s.adaptationTime = t;
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
    end
end
