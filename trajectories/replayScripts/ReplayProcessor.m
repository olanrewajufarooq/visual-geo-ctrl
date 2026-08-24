classdef ReplayProcessor
    %REPLAYPROCESSOR Public compatibility facade for replay processing.
    %   The implementation lives in ReplayProcessingCore so loading and
    %   preprocessing concerns remain separate from the public API.

    methods (Static)
        function summary = processAll(varargin)
            summary = ReplayProcessingCore.processAll(varargin{:});
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
