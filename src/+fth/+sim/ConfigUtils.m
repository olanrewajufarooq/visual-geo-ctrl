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
                    out = gammaValue(index, 1);
                end
            else
                out = fth.sim.ConfigUtils.selectRow(gammaValue, index);
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
            %   value: the gain array to validate.
            %   fieldName: name used in error messages.
            %   expectedRows: expected number of elements / columns.
            if isempty(value), return; end
            isValidVector = isvector(value) && numel(value) == expectedRows;
            isValidMatrix = ismatrix(value) && size(value,2) == expectedRows;
            if ~(isValidVector || isValidMatrix)
                error('Config:InvalidGainShape', ...
                    '%s must be a %dx1 vector, 1x%d vector, or Nx%d matrix.', ...
                    fieldName, expectedRows, expectedRows, expectedRows);
            end
        end

        function validateGammaShape(gamma, adaptMode)
            %VALIDATEGAMMASHAPE Validate adaptive gain shape for the selected mode.
            if isempty(gamma), return; end
            value = gamma;
            if isempty(adaptMode), adaptMode = 'none'; end
            if strcmpi(adaptMode, 'bregman')
                isScalarBatch = isnumeric(value) && ...
                    (isscalar(value) || (ismatrix(value) && size(value,2) == 1 && numel(value) ~= 10));
                if ~isScalarBatch || any(~isfinite(value(:))) || any(value(:) <= 0)
                    error('Config:InvalidGainShape', ...
                        'Bregman Gamma must be a positive scalar or Nx1 positive scalar batch; 10-element gain vectors are not valid for Bregman.');
                end
                return;
            end
            isScalar = isnumeric(value) && isscalar(value);
            isValidVector = isnumeric(value) && isvector(value) && numel(value) == 10;
            isValidMatrix = isnumeric(value) && ismatrix(value) && size(value,2) == 10;
            if ~(isScalar || isValidVector || isValidMatrix)
                error('Config:InvalidGainShape', ...
                    'Gamma must be scalar, a 10x1 vector, 1x10 vector, or Nx10 matrix.');
            end
        end

        function validateTrajectoryBatch(trajNames, trajHover)
            %VALIDATETRAJECTORYBATCH Validate trajectory batch configuration.
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

        function count = gainBatchCount(value, expectedRows)
            %GAINBATCHCOUNT Return 1 or N based on matrix row count.
            count = 1;
            if isempty(value), return; end
            fth.sim.ConfigUtils.validateGainShape(value, 'gain', expectedRows);
            if ismatrix(value) && size(value,2) == expectedRows && size(value,1) > 1
                count = size(value,1);
            end
        end

        function count = gammaBatchCount(gamma, adaptMode)
            %GAMMABATCHCOUNT Return number of adaptive gain rows.
            count = 1;
            if isempty(gamma), return; end
            fth.sim.ConfigUtils.validateGammaShape(gamma, adaptMode);
            value = gamma;
            if strcmpi(adaptMode, 'bregman') && ...
                    ismatrix(value) && size(value,2) == 1 && size(value,1) > 1
                count = size(value,1);
            elseif ~strcmpi(adaptMode, 'bregman') && ...
                    ismatrix(value) && size(value,2) == 10 && size(value,1) > 1
                count = size(value,1);
            end
        end

        function M = resolveSimBatchCount(batchNames, fields)
            %RESOLVESIMBATCHCOUNT Return M, the number of named sim runs.
            %   batchNames: cell array of strings (from sim.batchNames), may be empty.
            %   fields: cell array of values from batched option fields.
            %   If batchNames is non-empty: M = numel(batchNames).
            %   Else: M = max row count among all fields that have >1 row.
            if ~isempty(batchNames)
                M = numel(batchNames);
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
