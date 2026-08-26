classdef ReplayProcessor
    %REPLAYPROCESSOR Public compatibility facade for replay processing.
    %   The implementation lives in ReplayProcessingCore so loading and
    %   preprocessing concerns remain separate from the public API.

    methods (Static)
        function summary = processAll(varargin)
            isConvenienceCall = nargin >= 1 && nargin <= 2 && ...
                (islogical(varargin{1}) || isnumeric(varargin{1}));
            if isConvenienceCall
                trajectoryIds = [];
                if nargin == 2
                    trajectoryIds = varargin{2};
                end
                summary = ReplayProcessingCore.processAll( ...
                    [], [], [], varargin{1}, trajectoryIds);
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
