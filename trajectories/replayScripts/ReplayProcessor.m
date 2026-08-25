classdef ReplayProcessor
    %REPLAYPROCESSOR Public compatibility facade for replay processing.
    %   The implementation lives in ReplayProcessingCore so loading and
    %   preprocessing concerns remain separate from the public API.

    methods (Static)
        function summary = processAll(varargin)
            isConvenienceCall = nargin >= 2 && nargin <= 3 && ...
                (ischar(varargin{1}) || isstring(varargin{1})) && ...
                ismember(lower(char(string(varargin{1}))), {'wnoj', 'poly'});
            if isConvenienceCall
                trajectoryIds = [];
                if nargin == 3
                    trajectoryIds = varargin{3};
                end
                summary = ReplayProcessingCore.processAll( ...
                    [], [], [], varargin{1}, varargin{2}, trajectoryIds);
            else
                summary = ReplayProcessingCore.processAll(varargin{:});
            end
        end

        function entry = loadEntry(varargin)
            entry = ReplayProcessingCore.loadEntry(varargin{:});
        end

        function traj = loadArtifact(varargin)
            traj = ReplayProcessingCore.loadArtifact(varargin{:});
        end

        function limits = defaultAxisLimits(varargin)
            limits = ReplayProcessingCore.defaultAxisLimits(varargin{:});
        end

        function rootDir = defaultRootDir(varargin)
            rootDir = ReplayProcessingCore.defaultRootDir(varargin{:});
        end
    end
end
