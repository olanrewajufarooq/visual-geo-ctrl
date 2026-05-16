classdef ConfigUtils
    %CONFIGUTILS Static utility helpers for fth.sim.Config.
    %   All methods are static — no instance needed.

    methods (Static)

        function out = selectRow(value, index)
            %SELECTROW Return a column vector for the given row index, or
            %   broadcast if scalar/1-row.
            if isvector(value)
                out = value(:);
            elseif size(value,1) == 1
                out = value(:);
            else
                out = value(index,:).';
            end
        end

        function out = selectGammaRow(adaptMode, gammaValue, index)
            %SELECTGAMMAROW Select or broadcast Gamma for a run index.
            %   adaptMode: 'bregman' or other string.
            if strcmpi(adaptMode, 'bregman')
                if isscalar(gammaValue)
                    out = gammaValue;
                else
                    out = gammaValue(index);   % linear index — works for row or column vector
                end
            else
                out = fth.sim.ConfigUtils.selectRow(gammaValue, index);
            end
        end

        function [canonical, M] = normalizeBatchField(value, singleWidth)
            %NORMALIZEBATCHFIELD Canonicalize a batch field to M×singleWidth form.
            %   value      : raw input (scalar, vector, or matrix).
            %   singleWidth: elements per single-run config.
            %                  1  — scalar-per-run (Bregman Gamma, mass, dropTime)
            %                  3  — vector-per-run (CoG)
            %                  6  — vector-per-run (Kp, Kd)
            %                 10  — vector-per-run (Euclidean Gamma)
            %
            %   canonical  : M×singleWidth matrix in row-major batch form.
            %                When M=1 and singleWidth>1, returns a singleWidth×1 column.
            %                When singleWidth=1, returns an M×1 column (scalar if M=1).
            %   M          : number of batch runs represented by value.
            %
            %   Accepted layouts:
            %     singleWidth=1 : scalar → M=1; any vector (row or col) → M=numel
            %     singleWidth=W : W-element vector (any orientation) → M=1
            %                     N×W matrix → M=N  (canonical)
            %                     W×N matrix (N≠W) → transposed to N×W, M=N
            if isempty(value)
                canonical = value;
                M = 1;
                return;
            end
            if singleWidth == 1
                canonical = value(:);       % always column
                M = numel(value);
            else
                if isvector(value) && numel(value) == singleWidth
                    canonical = value(:);   % single config as column
                    M = 1;
                elseif size(value,2) == singleWidth
                    canonical = value;      % already N×W
                    M = size(value, 1);
                elseif size(value,1) == singleWidth
                    % W×N transposed layout — flip to N×W
                    canonical = value.';
                    M = size(value, 2);
                else
                    error('ConfigUtils:InvalidBatchShape', ...
                        'Cannot resolve batch size: value is %dx%d but singleWidth=%d.', ...
                        size(value,1), size(value,2), singleWidth);
                end
            end
        end

        function names = normalizeNames(name)
            %NORMALIZENAMES Normalize trajectory input into a cellstr.
            if ischar(name) || (isstring(name) && isscalar(name))
                names = {char(string(name))};
            elseif isstring(name)
                names = cellstr(name(:));
            elseif iscell(name)
                names = cellfun(@char, cellfun(@string, name(:), 'UniformOutput', false), 'UniformOutput', false);
            else
                error('Config:InvalidTrajectoryInput', ...
                    'Trajectory must be a char, string scalar, string array, or cell array.');
            end
            names = cellfun(@strtrim, names, 'UniformOutput', false);
            if any(cellfun(@isempty, names))
                error('Config:InvalidTrajectoryInput', 'Trajectory names cannot be empty.');
            end
        end

        function hover = normalizeHover(goToHoverBeforePathStarts, count)
            %NORMALIZEHOVER Normalize trajectory hover input.
            if isscalar(goToHoverBeforePathStarts)
                hover = repmat(logical(goToHoverBeforePathStarts), 1, count);
                return;
            end
            hover = logical(goToHoverBeforePathStarts(:)).';
            if numel(hover) ~= count
                error('Config:InvalidTrajectoryHover', ...
                    'Trajectory goToHoverBeforePathStarts must be a scalar or match the number of trajectories.');
            end
        end

        function folderName = trajectoryFolderName(name, index)
            %TRAJECTORYFOLDERNAME Build a compact trajectory folder name.
            shortName = fth.io.NamingUtils.trajectoryLabel(name);
            folderName = sprintf('t%02d_%s', index, shortName);
        end

        function validateGainShape(value, fieldName, expectedRows)
            %VALIDATEGAINSHAPE Validate the shape of a gain value.
            %   Accepts W-element vectors (any orientation), N×W matrices, and
            %   W×N transposed matrices (all resolved via normalizeBatchField).
            if isempty(value), return; end
            isValidVector   = isvector(value) && numel(value) == expectedRows;
            isValidMatrix   = ismatrix(value) && size(value,2) == expectedRows;
            isTransposed    = ismatrix(value) && size(value,1) == expectedRows;
            if ~(isValidVector || isValidMatrix || isTransposed)
                error('Config:InvalidGainShape', ...
                    '%s must be a %d-element vector, Nx%d matrix, or %dxN transposed matrix.', ...
                    fieldName, expectedRows, expectedRows, expectedRows);
            end
        end

        function validateGammaShape(gamma, adaptMode)
            %VALIDATEGAMMASHAPE Validate adaptive gain shape for the selected mode.
            %   Accepts both row and column vector orientations (and transposed
            %   matrices for Euclidean mode). Zero is allowed for Bregman batch
            %   entries where it signals "no adaptation for this run."
            %   Cell Gamma is validated per-element by the caller; skip here.
            if isempty(gamma), return; end
            if iscell(gamma), return; end
            value = gamma;
            if isempty(adaptMode), adaptMode = 'none'; end
            if strcmpi(adaptMode, 'bregman')
                % Any numeric vector (scalar, row, or column) is valid.
                if ~isnumeric(value) || ~isvector(value)
                    error('Config:InvalidGainShape', ...
                        'Bregman Gamma must be a positive scalar or a vector of scalars (one per batch run; 0 = disabled).');
                end
                if any(~isfinite(value(:))) || any(value(:) < 0)
                    error('Config:InvalidGainShape', ...
                        'Bregman Gamma values must be finite and non-negative (0 disables adaptation for that run).');
                end
                return;
            end
            % Euclidean / none: scalar, 10-element vector, N×10 matrix, or 10×N transposed.
            isScalar      = isnumeric(value) && isscalar(value);
            isValidVector = isnumeric(value) && isvector(value) && numel(value) == 10;
            isValidMatrix = isnumeric(value) && ismatrix(value) && size(value,2) == 10;
            isTransposed  = isnumeric(value) && ismatrix(value) && size(value,1) == 10;
            if ~(isScalar || isValidVector || isValidMatrix || isTransposed)
                error('Config:InvalidGainShape', ...
                    'Gamma must be a scalar, 10-element vector, Nx10 matrix, or 10xN transposed matrix.');
            end
        end

        function validateTrajectoryBatch(trajNames, trajHover)
            %VALIDATETRAJECTORYBATCH Validate trajectory batch configuration.
            %   trajNames: cell array of trajectory name strings.
            %   trajHover: logical array of hover flags, or [] when no hover override
            %              was set (from getTrajectoryBatchEntries hasHoverOverride=false).
            %   Precondition: pass trajHover=[] when there is no explicit hover override.
            if isempty(trajNames)
                error('Config:InvalidTrajectoryBatch', 'At least one trajectory must be configured.');
            end
            hasHoverOverride = ~isempty(trajHover);
            if hasHoverOverride && numel(trajHover) ~= numel(trajNames)
                error('Config:InvalidTrajectoryHover', ...
                    'Trajectory goToHoverBeforePathStarts must be a scalar or match the number of trajectories.');
            end
        end

        function Gamma = defaultGains(adaptMode)
            %DEFAULTGAINS Return adaptation-mode-specific Gamma default.
            if strcmpi(adaptMode, 'bregman')
                Gamma = 4e-3 * 20;
            else
                Gamma = 4e-3 * [20;20;30;1;1;1;90;30;30;60];
            end
        end

        function count = gainBatchCount(value, fieldName, expectedRows)
            %GAINBATCHCOUNT Return number of batch rows for a gain field.
            %   Handles all orientations via normalizeBatchField.
            count = 1;
            if isempty(value), return; end
            fth.sim.ConfigUtils.validateGainShape(value, fieldName, expectedRows);
            [~, count] = fth.sim.ConfigUtils.normalizeBatchField(value, expectedRows);
        end

        function count = gammaBatchCount(gamma, adaptMode)
            %GAMMABATCHCOUNT Return number of batch rows for an adaptive gain.
            %   Cell Gamma: each element is one run, so count = numel(gamma).
            count = 1;
            if isempty(gamma), return; end
            if iscell(gamma), count = numel(gamma); return; end
            fth.sim.ConfigUtils.validateGammaShape(gamma, adaptMode);
            singleWidth = 1;
            if ~strcmpi(adaptMode, 'bregman'), singleWidth = 10; end
            [~, count] = fth.sim.ConfigUtils.normalizeBatchField(gamma, singleWidth);
        end

        function M = resolveSimBatchCount(runNames, fields)
            %RESOLVESIMBATCHCOUNT Resolve the number of named simulation runs (M).
            %   Called in Phase 2 batch standardization by expandBatchConfigs.
            %   runNames: cell array from sim.runNames, or {} if not set.
            %   fields: cell array of batched option values (Kp, Kd, Gamma, etc.).
            %   Returns M = numel(runNames) if names are set, else max row count.
            if ~isempty(runNames)
                M = numel(runNames);
                return;
            end
            M = 1;
            for i = 1:numel(fields)
                v = fields{i};
                if isempty(v), continue; end
                if ismatrix(v) && size(v,1) > 1
                    M = max(M, size(v,1));
                elseif iscell(v) && numel(v) > 1
                    M = max(M, numel(v));
                end
            end
        end

    end
end
