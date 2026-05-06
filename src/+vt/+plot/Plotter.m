classdef Plotter < handle
    %PLOTTER Generate live and summary plots for simulations.
    %   Supports live updating layouts, summary figures, and standalone
    %   stacked plots. Works with log structs from vt.core.Logger.
    %
    %   Plot outputs are saved under results/<run-id>/ by default.
    properties
        outDir
        savePng
        lineWidth
        rgb
        duration
    end

    properties (Access = private)
        liveAxes
        liveUseUrdf
        liveLayoutType
        titleFontSize
        labelFontSize
        legendFontSize
        plotWidthLarge
        plotWidthSmall
        plotWidthPaper
        plotWidthPaperWide
        plotHeightLarge
        plotHeightMedium
        plotHeightSmall
        plotHeightStackedTwoRow
        plotMarginLarge
        plotMarginSmall
        plotGapLarge
        plotGapSmall
        plotGapStacked
        plotGapStackedSummary
    end

    methods
        function obj = Plotter(outDir, opts)
            %PLOTTER Configure plotting output and styling options.
            %   Inputs:
            %     outDir - output directory for saved plots.
            %     opts - struct with fields: savePng, duration.
            %
            %   Output:
            %     obj - Plotter instance.
            if nargin < 2
                opts = struct();
            end
            obj.outDir = outDir;
            obj.savePng = isfield(opts,'savePng') && opts.savePng;
            obj.duration = [];
            if isfield(opts, 'duration')
                obj.duration = opts.duration;
            end
            obj.lineWidth = 2.0;
            obj.rgb = [
                1.00 0.00 0.00;
                0.00 1.00 0.00;
                0.00 0.00 1.00;
            ];
            % Font styling (axes tick+labels share labelFontSize).
            obj.titleFontSize = 13;
            obj.labelFontSize = 12;
            obj.legendFontSize = 12;
            obj.plotWidthLarge = 1200;
            obj.plotWidthSmall = 800;
            obj.plotWidthPaper = 500;
            obj.plotWidthPaperWide = 900;
            obj.plotHeightLarge = 400;
            obj.plotHeightMedium = 300;
            obj.plotHeightSmall = 200;
            obj.plotHeightStackedTwoRow = round(2.4 * obj.plotHeightSmall);
            obj.plotMarginLarge = [0.06 0.10 0.03 0.12];
            obj.plotMarginSmall = [0.06 0.11 0.03 0.12];
            % Gaps are [gv gh] = [verticalGap horizontalGap]
            obj.plotGapLarge = [0.13 0.05];
            obj.plotGapSmall = [0.05 0.03];
            % Extra vertical breathing room for 2-row stacked figures.
            obj.plotGapStacked = [0.07 0.03];
            obj.plotGapStackedSummary = 0.06;
        end

        function saveFigure(obj, fig, filename)
            %SAVEFIGURE Save a figure using configured options.
            %   Inputs:
            %     fig - figure handle.
            %     filename - file base name (no extension).
            obj.saveFigureInternal(fig, filename);
        end

        function fig = plotLiveNominal(obj, logs, fig, useUrdfSlot, layoutType)
            %PLOTLIVENOMINAL Render or update live nominal layout.
            %   Inputs:
            %     logs - logger struct (may be empty during initialization).
            %     fig - existing figure handle (optional).
            %     useUrdfSlot - reserve axes for URDF view (optional).
            %     layoutType - 'row-major' or 'column-major' (optional).
            %
            %   Output:
            %     fig - live view figure handle.
            if nargin < 4
                useUrdfSlot = false;
            end
            if nargin < 5
                layoutType = [];
            end
            layoutType = obj.normalizeLayoutType(layoutType);
            if nargin < 3 || isempty(fig) || ~isvalid(fig)
                fig = figure('Name','Live View','Position',[50 50 1400 600]);
                obj.liveAxes = struct();
            end

            if useUrdfSlot
                obj.ensureLiveLayout(fig, useUrdfSlot, layoutType);
                if nargin >= 2 && isstruct(logs) && isfield(logs, 't') && ~isempty(logs.t)
                    obj.renderLiveAxes(logs, useUrdfSlot);
                    sgtitle(fig, sprintf('Live View - t = %.2f s', logs.t(end)));
                else
                    sgtitle(fig, 'Live View');
                end
            else
                clf(fig);
                obj.drawNominalLayout(fig, logs, layoutType);
                if nargin >= 2 && isstruct(logs) && isfield(logs, 't') && ~isempty(logs.t)
                    sgtitle(fig, sprintf('Live View - t = %.2f s', logs.t(end)));
                end
            end
        end

        function fig = plotLiveAdaptive(obj, logs, est, fig, useUrdfSlot, layoutType)
            %PLOTLIVEADAPTIVE Render or update live adaptive layout.
            %   Inputs:
            %     logs - logger struct.
            %     est - estimation struct (optional).
            %     fig - existing figure handle (optional).
            %     useUrdfSlot - reserve axes for URDF view (optional).
            %     layoutType - 'row-major' or 'column-major' (optional).
            %
            %   Output:
            %     fig - live view figure handle.
            if nargin < 5
                useUrdfSlot = false;
            end
            if nargin < 6
                layoutType = [];
            end
            layoutType = obj.normalizeLayoutType(layoutType);
            if nargin < 4 || isempty(fig) || ~isvalid(fig)
                fig = figure('Name','Live View','Position',[50 50 1.25*obj.plotWidthLarge 2.25*obj.plotHeightLarge]);
                obj.liveAxes = struct();
            end

            if useUrdfSlot
                obj.ensureLiveLayoutAdaptive(fig, useUrdfSlot, layoutType);
                if nargin >= 2 && isstruct(logs) && isfield(logs, 't') && ~isempty(logs.t)
                    if nargin < 3 || isempty(est)
                        est = [];
                    end
                    obj.renderLiveAxesAdaptive(logs, est, useUrdfSlot);
                    sgtitle(fig, sprintf('Live View - t = %.2f s', logs.t(end)));
                else
                    sgtitle(fig, 'Live View');
                end
            else
                clf(fig);
                obj.drawAdaptiveLayout(fig, logs, est, layoutType);
                if nargin >= 2 && isstruct(logs) && isfield(logs, 't') && ~isempty(logs.t)
                    sgtitle(fig, sprintf('Live View - t = %.2f s', logs.t(end)));
                end
            end
        end

        function ax = getLiveUrdfAxes(obj)
            %GETLIVEURDFAXES Return axes handle reserved for URDF view.
            %   Output:
            %     ax - axes handle or empty.
            ax = [];
            if isstruct(obj.liveAxes) && isfield(obj.liveAxes, 'urdf') && isgraphics(obj.liveAxes.urdf)
                ax = obj.liveAxes.urdf;
            end
        end

        function fig = plotSummaryNominal(obj, logs, fig, ~)
            %PLOTSUMMARYNOMINAL Plot final summary for nominal run.
            %   Inputs:
            %     logs - logger struct.
            %     fig - existing figure handle (optional).
            %     layoutType - 'row-major' or 'column-major' (optional).
            %
            %   Output:
            %     fig - summary figure handle.
            if nargin < 3 || isempty(fig) || ~isvalid(fig)
                fig = figure('Name','Final Summary - Nominal', ...
                    'Position', [100 100 obj.plotWidthPaperWide + obj.plotWidthPaper round(3.35 * obj.plotHeightSmall)]);
            else
                clf(fig);
            end

            obj.drawNominalSummaryLayout(fig, logs);
            obj.saveFigureInternal(fig, 'summary_nominal');
        end

        function fig = plotSummaryAdaptive(obj, logs, est, fig, ~)
            %PLOTSUMMARYADAPTIVE Plot final summary for adaptive run.
            %   Inputs:
            %     logs - logger struct.
            %     est - estimation struct.
            %     fig - existing figure handle (optional).
            %     layoutType - 'row-major' or 'column-major' (optional).
            %
            %   Output:
            %     fig - summary figure handle.
            if nargin < 4 || isempty(fig) || ~isvalid(fig)
                fig = figure('Name','Final Summary - Adaptive', ...
                    'Position', [50 50 obj.plotWidthPaperWide + 2*obj.plotWidthPaper round(4.35 * obj.plotHeightSmall)]);
            else
                clf(fig);
            end

            obj.drawAdaptiveSummaryLayout(fig, logs, est);
            obj.saveFigureInternal(fig, 'summary_adaptive');
        end

        function plotStandaloneSubplotsNominal(obj, logs)
            %PLOTSTANDALONESUBLOTSNOMINAL Save separate nominal plots.
            %   Input:
            %     logs - logger struct with nominal signals.
            fig = figure('Name','Standalone - 3D Path', ...
                'Position', [100 100 obj.plotWidthPaperWide round(2.0 * obj.plotHeightMedium)]);
            obj.plot3DView(logs);
            obj.saveFigureInternal(fig, 'standalone_3d');

            fig = figure('Name','Standalone - XY Path', ...
                'Position', [100 100 obj.plotWidthPaperWide round(1.5 * obj.plotHeightMedium)]);
            obj.plotXYView(logs);
            obj.saveFigureInternal(fig, 'standalone_xy');

            fig = figure('Name','Standalone - Altitude', ...
                'Position', [100 100 obj.plotWidthPaper obj.plotHeightSmall]);
            obj.plotZvsT(logs);
            obj.saveFigureInternal(fig, 'standalone_z');

            fig = figure('Name','Standalone - Position & Orientation', ...
                'Position', [100 100 obj.plotWidthPaper obj.plotHeightStackedTwoRow]);
            ax = obj.createVerticalAxes(fig, 2, obj.plotMarginSmall, obj.plotGapStacked);
            axes(ax(1)); obj.plotPosition(logs);
            axes(ax(2)); obj.plotOrientation(logs);
            obj.finalizeStackedAxes(ax, 'Time [s]');
            obj.saveFigureInternal(fig, 'standalone_pos_orient');

            fig = figure('Name','Standalone - Velocity', ...
                'Position', [100 100 obj.plotWidthPaper obj.plotHeightStackedTwoRow]);
            ax = obj.createVerticalAxes(fig, 2, obj.plotMarginSmall, obj.plotGapStacked);
            axes(ax(1)); obj.plotLinearVel(logs);
            axes(ax(2)); obj.plotAngularVel(logs);
            obj.finalizeStackedAxes(ax, 'Time [s]');
            obj.saveFigureInternal(fig, 'standalone_vel');

            fig = figure('Name','Standalone - Wrench', ...
                'Position', [100 100 obj.plotWidthPaper obj.plotHeightStackedTwoRow]);
            ax = obj.createVerticalAxes(fig, 2, obj.plotMarginSmall, obj.plotGapStacked);
            axes(ax(1)); obj.plotForce(logs);
            axes(ax(2)); obj.plotTorque(logs);
            obj.finalizeStackedAxes(ax, 'Time [s]');
            obj.saveFigureInternal(fig, 'standalone_wrench');
        end

        function plotStandaloneSubplotsAdaptive(obj, logs, est)
            %PLOTSTANDALONESUBPLOTSADAPTIVE Save separate adaptive plots.
            %   Inputs:
            %     logs - logger struct.
            %     est - estimation struct.
            obj.plotStandaloneSubplotsNominal(logs);

            fig = figure('Name','Standalone - Mass & CoG', ...
                'Position', [100 100 obj.plotWidthPaper obj.plotHeightStackedTwoRow]);
            ax = obj.createVerticalAxes(fig, 2, obj.plotMarginSmall, obj.plotGapStacked);
            axes(ax(1)); obj.plotMass(est);
            axes(ax(2)); obj.plotCoG(est);
            obj.finalizeStackedAxes(ax, 'Time [s]');
            obj.saveFigureInternal(fig, 'standalone_mass_cog');

            fig = figure('Name','Standalone - Principal Inertia', ...
                'Position', [100 100 obj.plotWidthPaper obj.plotHeightSmall]);
            obj.plotPrincipalInertia(est);
            obj.saveFigureInternal(fig, 'standalone_inertia_principal');

            fig = figure('Name','Standalone - Off-diag Inertia', ...
                'Position', [100 100 obj.plotWidthPaper obj.plotHeightSmall]);
            obj.plotOffDiagInertia(est);
            obj.saveFigureInternal(fig, 'standalone_inertia_offdiag');
        end

        function plotStackedAllState(obj, logs)
            %PLOTSTACKEDALLSTATE Plot stacked state and wrench signals.
            %   Input:
            %     logs - logger struct.
            fig = figure('Name','Stacked - States', ...
                'Position', [100 100 obj.plotWidthPaper 6*obj.plotHeightSmall]);
            ax = obj.createVerticalAxes(fig, 6, [0.06 0.11 0.03 0.12], [0.035 0.05]);
            axes(ax(1)); obj.plotPosition(logs);
            axes(ax(2)); obj.plotOrientation(logs);
            axes(ax(3)); obj.plotLinearVel(logs);
            axes(ax(4)); obj.plotAngularVel(logs);
            axes(ax(5)); obj.plotForce(logs);
            axes(ax(6)); obj.plotTorque(logs);
            obj.finalizeStackedAxes(ax, 'Time [s]');
            obj.saveFigureInternal(fig, 'stack_all_state');
        end

        function plotStackedPositionOrientation(obj, logs)
            %PLOTSTACKEDPOSITIONORIENTATION Plot stacked pose signals.
            %   Input:
            %     logs - logger struct.
            fig = figure('Name','Stacked - Position & Orientation', ...
                'Position', [100 100 obj.plotWidthPaper obj.plotHeightStackedTwoRow]);
            ax = obj.createVerticalAxes(fig, 2, obj.plotMarginSmall, obj.plotGapStacked);
            axes(ax(1)); obj.plotPosition(logs);
            axes(ax(2)); obj.plotOrientation(logs);
            obj.finalizeStackedAxes(ax, 'Time [s]');
            obj.saveFigureInternal(fig, 'stack_pos_orient');
        end

        function plotStackedVelocity(obj, logs)
            %PLOTSTACKEDVELOCITY Plot stacked velocity signals.
            %   Input:
            %     logs - logger struct.
            fig = figure('Name','Stacked - Velocity', ...
                'Position', [100 100 obj.plotWidthPaper obj.plotHeightStackedTwoRow]);
            ax = obj.createVerticalAxes(fig, 2, obj.plotMarginSmall, obj.plotGapStacked);
            axes(ax(1)); obj.plotLinearVel(logs);
            axes(ax(2)); obj.plotAngularVel(logs);
            obj.finalizeStackedAxes(ax, 'Time [s]');
            obj.saveFigureInternal(fig, 'stack_vel');
        end

        function plotStackedWrench(obj, logs)
            %PLOTSTACKEDWRENCH Plot stacked force/torque signals.
            %   Input:
            %     logs - logger struct.
            fig = figure('Name','Stacked - Force & Torque', ...
                'Position', [100 100 obj.plotWidthPaper obj.plotHeightStackedTwoRow]);
            ax = obj.createVerticalAxes(fig, 2, obj.plotMarginSmall, obj.plotGapStacked);
            axes(ax(1)); obj.plotForce(logs);
            axes(ax(2)); obj.plotTorque(logs);
            obj.finalizeStackedAxes(ax, 'Time [s]');
            obj.saveFigureInternal(fig, 'stack_wrench');
        end

        function plotStackedEstimation(obj, est)
            %PLOTSTACKEDESTIMATION Plot stacked estimation signals.
            %   Input:
            %     est - estimation struct.
            fig = figure('Name','Stacked - Mass, CoG, Inertia', ...
                'Position', [100 100 obj.plotWidthPaper 4*obj.plotHeightSmall]);
            ax = obj.createVerticalAxes(fig, 4);
            axes(ax(1)); obj.plotMass(est);
            axes(ax(2)); obj.plotCoG(est);
            axes(ax(3)); obj.plotPrincipalInertia(est);
            axes(ax(4)); obj.plotOffDiagInertia(est);
            obj.finalizeStackedAxes(ax, 'Time [s]');
            obj.saveFigureInternal(fig, 'stack_estimation');
        end

        function plotStackedInertia(obj, est)
            %PLOTSTACKEDINERTIA Plot stacked inertia estimates.
            %   Input:
            %     est - estimation struct.
            fig = figure('Name','Stacked - Inertia', ...
                'Position', [100 100 obj.plotWidthPaper obj.plotHeightStackedTwoRow]);
            ax = obj.createVerticalAxes(fig, 2, obj.plotMarginSmall, obj.plotGapStacked);
            axes(ax(1)); obj.plotPrincipalInertia(est);
            axes(ax(2)); obj.plotOffDiagInertia(est);
            obj.finalizeStackedAxes(ax, 'Time [s]');
            obj.saveFigureInternal(fig, 'stack_inertia');
        end
    end

    methods (Access = private)
        function drawNominalLayout(obj, fig, logs, layoutType)
            %DRAWNOMINALLAYOUT Draw nominal summary layout.
            %   Inputs:
            %     fig - figure handle.
            %     logs - logger struct.
            %     layoutType - 'row-major' or 'column-major'.
            layoutType = obj.normalizeLayoutType(layoutType);
            clf(fig);

            if strcmp(layoutType, 'column-major')
                posGridT = obj.buildGridPositions(3, 2, obj.plotMarginLarge, obj.plotGapLarge);
                posList = cell(1, 6);
                for i = 1:6
                    r = ceil(i / 3);
                    c = i - (r - 1) * 3;
                    posList{i} = posGridT{c, r};
                end
            else
                posGrid = obj.buildGridPositions(2, 3, obj.plotMarginLarge, obj.plotGapLarge);
                posList = obj.orderPositions(posGrid, 'row-major');
            end

            subplot('Position', posList{1}); obj.plot3DView(logs);
            subplot('Position', posList{2}); obj.plotXYView(logs);
            subplot('Position', posList{3}); obj.plotZvsT(logs);

            obj.plotStacked(@obj.plotPosition, @obj.plotOrientation, posList{4}, logs);
            obj.plotStacked(@obj.plotLinearVel, @obj.plotAngularVel, posList{5}, logs);
            obj.plotStacked(@obj.plotForce, @obj.plotTorque, posList{6}, logs);
        end

        function drawAdaptiveLayout(obj, fig, logs, est, layoutType)
            %DRAWADAPTIVELAYOUT Draw adaptive summary layout.
            %   Inputs:
            %     fig - figure handle.
            %     logs - logger struct.
            %     est - estimation struct.
            %     layoutType - 'row-major' or 'column-major'.
            layoutType = obj.normalizeLayoutType(layoutType);
            clf(fig);

            posGrid = obj.buildGridPositions(3, 3, obj.plotMarginLarge, obj.plotGapLarge);
            posList = obj.orderPositions(posGrid, layoutType);

            subplot('Position', posList{1}); obj.plot3DView(logs);
            subplot('Position', posList{2}); obj.plotXYView(logs);
            subplot('Position', posList{3}); obj.plotZvsT(logs);

            obj.plotStacked(@obj.plotPosition, @obj.plotOrientation, posList{4}, logs);
            obj.plotStacked(@obj.plotLinearVel, @obj.plotAngularVel, posList{5}, logs);
            obj.plotStacked(@obj.plotForce, @obj.plotTorque, posList{6}, logs);

            obj.plotStackedEst(@obj.plotMass, @obj.plotCoG, posList{7}, est);
            subplot('Position', posList{8}); obj.plotPrincipalInertia(est);
            subplot('Position', posList{9}); obj.plotOffDiagInertia(est);
        end

        function drawNominalSummaryLayout(obj, fig, logs)
            %DRAWNOMINALSUMMARYLAYOUT Draw publication summary for nominal run.
            clf(fig);
            posGrid = obj.buildWeightedGridPositions([1 1 1], [0.75 1.0], ...
                [0.05 0.07 0.03 0.07], [0.06 0.07]);
            posList = obj.orderPositions(posGrid, 'row-major');

            subplot('Position', posList{1}); obj.plot3DView(logs);
            subplot('Position', posList{3}); obj.plotXYView(logs);
            subplot('Position', posList{5}); obj.plotZvsT(logs);

            obj.plotStacked(@obj.plotPosition, @obj.plotOrientation, posList{2}, logs);
            obj.plotStacked(@obj.plotLinearVel, @obj.plotAngularVel, posList{4}, logs);
            obj.plotStacked(@obj.plotForce, @obj.plotTorque, posList{6}, logs);
        end

        function drawAdaptiveSummaryLayout(obj, fig, logs, est)
            %DRAWADAPTIVESUMMARYLAYOUT Draw publication summary for adaptive run.
            clf(fig);
            posGrid = obj.buildWeightedGridPositions([1 1 1], [0.75 1.0 1.0], ...
                [0.05 0.07 0.03 0.07], [0.06 0.06]);
            posList = obj.orderPositions(posGrid, 'row-major');

            subplot('Position', posList{1}); obj.plot3DView(logs);
            subplot('Position', posList{4}); obj.plotXYView(logs);
            subplot('Position', posList{7}); obj.plotZvsT(logs);

            obj.plotStacked(@obj.plotPosition, @obj.plotOrientation, posList{2}, logs, est);
            obj.plotStacked(@obj.plotLinearVel, @obj.plotAngularVel, posList{5}, logs, est);
            obj.plotStacked(@obj.plotForce, @obj.plotTorque, posList{8}, logs, est);

            estPos = obj.mergePositions(posList{3}, posList{9});
            ax = obj.createAxesInPosition(fig, estPos, 4, [0.02 0.04 0.02 0.04], 0.05);
            axes(ax(1)); obj.plotMass(est);
            axes(ax(2)); obj.plotCoG(est);
            axes(ax(3)); obj.plotPrincipalInertia(est);
            axes(ax(4)); obj.plotOffDiagInertia(est);
            obj.finalizeStackedAxes(ax, 'Time [s]');
        end

        function ensureLiveLayout(obj, fig, useUrdfSlot, layoutType)
            %ENSURELIVELAYOUT Build live axes for nominal runs.
            %   Inputs:
            %     fig - figure handle.
            %     useUrdfSlot - true to reserve a URDF axes slot.
            %     layoutType - layout string.
            layoutType = obj.normalizeLayoutType(layoutType);
            if isstruct(obj.liveAxes) && isfield(obj.liveAxes, 'xy') && isgraphics(obj.liveAxes.xy)
                if isequal(obj.liveUseUrdf, useUrdfSlot) && strcmp(obj.liveLayoutType, layoutType)
                    return;
                end
            end

            obj.liveUseUrdf = useUrdfSlot;
            obj.liveLayoutType = layoutType;
            obj.liveAxes = struct();
            clf(fig);

            if strcmp(layoutType, 'column-major')
                posGridT = obj.buildGridPositions(3, 2, obj.plotMarginLarge, obj.plotGapLarge);
                posList = cell(1, 6);
                for i = 1:6
                    r = ceil(i / 3);
                    c = i - (r - 1) * 3;
                    posList{i} = posGridT{c, r};
                end
            else
                posGrid = obj.buildGridPositions(2, 3, obj.plotMarginLarge, obj.plotGapLarge);
                posList = obj.orderPositions(posGrid, 'row-major');
            end

            if useUrdfSlot
                obj.liveAxes.urdf = axes('Parent', fig, 'Position', posList{1});
            else
                obj.liveAxes.view3d = axes('Parent', fig, 'Position', posList{1});
            end
            obj.liveAxes.xy = axes('Parent', fig, 'Position', posList{2});
            obj.liveAxes.z = axes('Parent', fig, 'Position', posList{3});

            [topPos, botPos] = obj.splitStacked(posList{4});
            obj.liveAxes.posTop = axes('Parent', fig, 'Position', topPos);
            obj.liveAxes.posBot = axes('Parent', fig, 'Position', botPos);

            [topPos, botPos] = obj.splitStacked(posList{5});
            obj.liveAxes.velTop = axes('Parent', fig, 'Position', topPos);
            obj.liveAxes.velBot = axes('Parent', fig, 'Position', botPos);

            [topPos, botPos] = obj.splitStacked(posList{6});
            obj.liveAxes.forceTop = axes('Parent', fig, 'Position', topPos);
            obj.liveAxes.forceBot = axes('Parent', fig, 'Position', botPos);
        end

        function ensureLiveLayoutAdaptive(obj, fig, useUrdfSlot, layoutType)
            %ENSURELIVELAYOUTADAPTIVE Build live axes for adaptive runs.
            %   Inputs:
            %     fig - figure handle.
            %     useUrdfSlot - true to reserve a URDF axes slot.
            %     layoutType - layout string.
            layoutType = obj.normalizeLayoutType(layoutType);
            if isstruct(obj.liveAxes) && isfield(obj.liveAxes, 'xy') && isgraphics(obj.liveAxes.xy)
                if isequal(obj.liveUseUrdf, useUrdfSlot) && strcmp(obj.liveLayoutType, layoutType)
                    return;
                end
            end

            obj.liveUseUrdf = useUrdfSlot;
            obj.liveLayoutType = layoutType;
            obj.liveAxes = struct();
            clf(fig);

            posGrid = obj.buildGridPositions(3, 3, obj.plotMarginLarge, obj.plotGapLarge);
            posList = obj.orderPositions(posGrid, layoutType);

            if useUrdfSlot
                obj.liveAxes.urdf = axes('Parent', fig, 'Position', posList{1});
            else
                obj.liveAxes.view3d = axes('Parent', fig, 'Position', posList{1});
            end
            obj.liveAxes.xy = axes('Parent', fig, 'Position', posList{2});
            obj.liveAxes.z = axes('Parent', fig, 'Position', posList{3});

            [topPos, botPos] = obj.splitStacked(posList{4});
            obj.liveAxes.posTop = axes('Parent', fig, 'Position', topPos);
            obj.liveAxes.posBot = axes('Parent', fig, 'Position', botPos);

            [topPos, botPos] = obj.splitStacked(posList{5});
            obj.liveAxes.velTop = axes('Parent', fig, 'Position', topPos);
            obj.liveAxes.velBot = axes('Parent', fig, 'Position', botPos);

            [topPos, botPos] = obj.splitStacked(posList{6});
            obj.liveAxes.forceTop = axes('Parent', fig, 'Position', topPos);
            obj.liveAxes.forceBot = axes('Parent', fig, 'Position', botPos);

            [topPos, botPos] = obj.splitStacked(posList{7});
            obj.liveAxes.estTop = axes('Parent', fig, 'Position', topPos);
            obj.liveAxes.estBot = axes('Parent', fig, 'Position', botPos);

            obj.liveAxes.inertiaP = axes('Parent', fig, 'Position', posList{8});
            obj.liveAxes.inertiaO = axes('Parent', fig, 'Position', posList{9});
        end

        function renderLiveAxes(obj, logs, useUrdfSlot)
            %RENDERLIVEAXES Update nominal live axes with latest logs.
            %   Inputs:
            %     logs - logger struct.
            %     useUrdfSlot - true when URDF occupies one subplot.
            if ~useUrdfSlot && isfield(obj.liveAxes, 'view3d') && isgraphics(obj.liveAxes.view3d)
                axes(obj.liveAxes.view3d);
                cla(obj.liveAxes.view3d);
                obj.plot3DView(logs, false);
            end

            axes(obj.liveAxes.xy);
            cla(obj.liveAxes.xy);
            obj.plotXYView(logs);

            axes(obj.liveAxes.z);
            cla(obj.liveAxes.z);
            obj.plotZvsT(logs);

            axes(obj.liveAxes.posTop);
            cla(obj.liveAxes.posTop);
            obj.plotPosition(logs);

            axes(obj.liveAxes.posBot);
            cla(obj.liveAxes.posBot);
            obj.plotOrientation(logs);

            axes(obj.liveAxes.velTop);
            cla(obj.liveAxes.velTop);
            obj.plotLinearVel(logs);

            axes(obj.liveAxes.velBot);
            cla(obj.liveAxes.velBot);
            obj.plotAngularVel(logs);

            axes(obj.liveAxes.forceTop);
            cla(obj.liveAxes.forceTop);
            obj.plotForce(logs);

            axes(obj.liveAxes.forceBot);
            cla(obj.liveAxes.forceBot);
            obj.plotTorque(logs);
        end

        function renderLiveAxesAdaptive(obj, logs, est, useUrdfSlot)
            %RENDERLIVEAXESADAPTIVE Update adaptive live axes with logs.
            %   Inputs:
            %     logs - logger struct.
            %     est - estimation struct.
            %     useUrdfSlot - true when URDF occupies one subplot.
            if ~useUrdfSlot && isfield(obj.liveAxes, 'view3d') && isgraphics(obj.liveAxes.view3d)
                axes(obj.liveAxes.view3d);
                cla(obj.liveAxes.view3d);
                obj.plot3DView(logs, false);
            end

            axes(obj.liveAxes.xy);
            cla(obj.liveAxes.xy);
            obj.plotXYView(logs);

            axes(obj.liveAxes.z);
            cla(obj.liveAxes.z);
            obj.plotZvsT(logs);

            axes(obj.liveAxes.posTop);
            cla(obj.liveAxes.posTop);
            obj.plotPosition(logs);

            axes(obj.liveAxes.posBot);
            cla(obj.liveAxes.posBot);
            obj.plotOrientation(logs);

            axes(obj.liveAxes.velTop);
            cla(obj.liveAxes.velTop);
            obj.plotLinearVel(logs);

            axes(obj.liveAxes.velBot);
            cla(obj.liveAxes.velBot);
            obj.plotAngularVel(logs);

            axes(obj.liveAxes.forceTop);
            cla(obj.liveAxes.forceTop);
            obj.plotForce(logs);

            axes(obj.liveAxes.forceBot);
            cla(obj.liveAxes.forceBot);
            obj.plotTorque(logs);

            if nargin >= 3 && ~isempty(est)
                axes(obj.liveAxes.estTop);
                cla(obj.liveAxes.estTop);
                obj.plotMass(est);

                axes(obj.liveAxes.estBot);
                cla(obj.liveAxes.estBot);
                obj.plotCoG(est);

                axes(obj.liveAxes.inertiaP);
                cla(obj.liveAxes.inertiaP);
                obj.plotPrincipalInertia(est);

                axes(obj.liveAxes.inertiaO);
                cla(obj.liveAxes.inertiaO);
                obj.plotOffDiagInertia(est);
            end
        end

        function plotStacked(obj, topFunc, botFunc, pos, logs, varargin)
            %PLOTSTACKED Helper to draw stacked nominal plots.
            %   Inputs:
            %     topFunc - function handle for top plot.
            %     botFunc - function handle for bottom plot.
            %     pos - subplot position.
            %     logs - logger struct.
            %     est - estimation struct (optional).
            [topPos, botPos] = obj.splitStacked(pos);

            est = [];
            if ~isempty(varargin)
                est = varargin{1};
            end
            
            if isstruct(est)
                subplot('Position', topPos); topFunc(logs, est);
                subplot('Position', botPos); botFunc(logs, est);
            else
                subplot('Position', topPos); topFunc(logs);
                subplot('Position', botPos); botFunc(logs);
            end
        end

        function plotStackedEst(obj, topFunc, botFunc, pos, est)
            %PLOTSTACKEDEST Helper to draw stacked estimate plots.
            %   Inputs:
            %     topFunc - function handle for top plot.
            %     botFunc - function handle for bottom plot.
            %     pos - subplot position.
            %     est - estimation struct.
            [topPos, botPos] = obj.splitStacked(pos);
            
            subplot('Position', topPos); topFunc(est);
            subplot('Position', botPos); botFunc(est);
        end

        function [topPos, botPos] = splitStacked(obj, pos)
            %SPLITSTACKED Split a subplot position into two rows.
            %   Input:
            %     pos - 1x4 position vector.
            %   Outputs:
            %     topPos - top subplot position.
            %     botPos - bottom subplot position.
            gap = obj.plotGapStackedSummary;
            h = pos(4) / 2 - gap / 2;
            topPos = [pos(1), pos(2) + h + gap, pos(3), h];
            botPos = [pos(1), pos(2), pos(3), h];
        end

        function posGrid = buildGridPositions(~, rows, cols, varargin)
            %BUILDGRIDPOSITIONS Build normalized subplot positions.
            %   Inputs:
            %     rows, cols - grid dimensions.
            %     Either:
            %       m - margins as [mt mb mr ml]
            %       g - gaps as [gv gh]
            %     Or legacy:
            %       ml,mr,mt,mb - margins.
            %       gh,gv - gaps between subplots.
            %   Output:
            %     posGrid - rows x cols cell of positions.
            if isempty(varargin)
                m = [0.08 0.08 0.02 0.06];
                g = [0.13 0.05];
            elseif numel(varargin) == 2
                m = varargin{1};
                g = varargin{2};
            elseif numel(varargin) == 6
                % Legacy: ml, mr, mt, mb, gh, gv
                ml = varargin{1};
                mr = varargin{2};
                mt = varargin{3};
                mb = varargin{4};
                gh = varargin{5};
                gv = varargin{6};
                m = [mt mb mr ml];
                g = [gv gh];
            else
                error('buildGridPositions:InvalidInputs', ...
                    'Expected (rows, cols), (rows, cols, m, g), or legacy (rows, cols, ml, mr, mt, mb, gh, gv).');
            end

            validateattributes(m, {'numeric'}, {'vector','numel',4,'>=',0,'<=',0.5});
            validateattributes(g, {'numeric'}, {'vector','numel',2,'>=',0,'<=',0.5});

            mt = m(1);
            mb = m(2);
            mr = m(3);
            ml = m(4);

            gv = g(1);
            gh = g(2);

            pw = (1 - ml - mr - (cols - 1) * gh) / cols;
            rh = (1 - mt - mb - (rows - 1) * gv) / rows;
            posGrid = cell(rows, cols);
            for r = 1:rows
                for c = 1:cols
                    x = ml + (c - 1) * (pw + gh);
                    y = mt + (rows - r) * (rh + gv);
                    posGrid{r, c} = [x, y, pw, rh];
                end
            end
        end

        function posList = orderPositions(~, posGrid, layoutType)
            %ORDERPOSITIONS Flatten grid by row- or column-major order.
            %   Inputs:
            %     posGrid - grid of positions.
            %     layoutType - 'row-major' or 'column-major'.
            %   Output:
            %     posList - linear cell array of positions.
            [rows, cols] = size(posGrid);
            posList = cell(1, rows * cols);
            idx = 1;
            if strcmp(layoutType, 'row-major')
                for r = 1:rows
                    for c = 1:cols
                        posList{idx} = posGrid{r, c};
                        idx = idx + 1;
                    end
                end
            else
                for c = 1:cols
                    for r = 1:rows
                        posList{idx} = posGrid{r, c};
                        idx = idx + 1;
                    end
                end
            end
        end

        function layoutType = normalizeLayoutType(~, layoutType)
            %NORMALIZELAYOUTTYPE Validate layout type string.
            %   Input:
            %     layoutType - requested layout (optional).
            %   Output:
            %     layoutType - validated layout string.
            if nargin < 2 || isempty(layoutType)
                layoutType = 'row-major';
                return;
            end
            layoutType = lower(char(layoutType));
            if ~strcmp(layoutType, 'row-major') && ~strcmp(layoutType, 'column-major')
                layoutType = 'row-major';
            end
        end

        function ax = createVerticalAxes(~, fig, nRows, m, g)
            %CREATEVERTICALAXES Create vertically stacked axes.
            %   Inputs:
            %     fig - figure handle.
            %     nRows - number of axes rows.
            %     m - margins as [mt mb mr ml] (optional).
            %     g - gaps as [gv gh] or scalar gv (optional).
            %   Output:
            %     ax - array of axes handles.
            if nargin < 4 || isempty(m)
                m = [0.06 0.11 0.04 0.12];
            end
            if nargin < 5 || isempty(g)
                g = 0.05;
            end

            validateattributes(m, {'numeric'}, {'vector','numel',4,'>=',0,'<=',0.5});
            validateattributes(g, {'numeric'}, {'nonempty'});

            if isscalar(g)
                gv = g;
            else
                validateattributes(g, {'numeric'}, {'vector','numel',2,'>=',0,'<=',0.5});
                gv = g(1);
            end

            mt = m(1);
            mb = m(2);
            mr = m(3);
            ml = m(4);
            totalH = 1 - mt - mb - (nRows - 1) * gv;
            h = totalH / nRows;
            ax = gobjects(nRows, 1);
            for i = 1:nRows
                y = 1 - mt - i * h - (i - 1) * gv;
                ax(i) = axes('Parent', fig, 'Position', [ml, y, 1 - ml - mr, h]);
                box(ax(i), 'on');
                ax(i).BoxStyle = 'full';
                ax(i).LineWidth = 1.5;
                grid(ax(i), 'off');
            end
        end

        function finalizeStackedAxes(obj, ax, xlabelText)
            %FINALIZESTACKEDAXES Finalize stacked axis labels and ticks.
            %   Inputs:
            %     ax - axes array.
            %     xlabelText - label for x-axis.
            if isempty(ax)
                return;
            end
            for i = 1:numel(ax)
                set(ax(i), 'FontSize', obj.labelFontSize);
                ax(i).Title.FontSize = obj.titleFontSize;
                ax(i).XLabel.FontSize = obj.labelFontSize;
                ax(i).YLabel.FontSize = obj.labelFontSize;
                ax(i).ZLabel.FontSize = obj.labelFontSize;
            end
            for i = 1:numel(ax) - 1
                set(ax(i), 'XTickLabel', []);
                xlabel(ax(i), '');
            end
            xlabel(ax(end), xlabelText);
        end

        function plot3DView(obj, logs, showEnd)
            %PLOT3DVIEW Plot desired vs actual 3D trajectory.
            %   Inputs:
            %     logs - logger struct.
            %     showEnd - true to mark end point (optional).
            if nargin < 3
                showEnd = false;
            end
            ax = gca; hold(ax, 'on');
            box(ax, 'on');
            grid(ax, 'on');
            ax.GridAlpha = 0.20;
            ax.GridLineStyle = ':';
            ax.LineWidth = 1.2;
            view(ax, 3);
            set(ax, 'FontSize', obj.labelFontSize);
            ax.Title.FontSize = obj.titleFontSize;
            ax.XLabel.FontSize = obj.labelFontSize;
            ax.YLabel.FontSize = obj.labelFontSize;
            ax.ZLabel.FontSize = obj.labelFontSize;
            title(ax, '3D Path');
            plot3(ax, logs.des.pos(:,1), logs.des.pos(:,2), logs.des.pos(:,3), 'k--', ...
                'LineWidth', obj.referenceLineWidth());
            plot3(ax, logs.actual.pos(:,1), logs.actual.pos(:,2), logs.actual.pos(:,3), '-', ...
                'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
            plot3(ax, logs.actual.pos(1,1), logs.actual.pos(1,2), logs.actual.pos(1,3), 'o', ...
                'MarkerSize', 7, 'Color', obj.rgb(3,:), 'MarkerFaceColor', obj.rgb(3,:));
            if showEnd
                plot3(ax, logs.actual.pos(end,1), logs.actual.pos(end,2), logs.actual.pos(end,3), 'o', ...
                    'MarkerSize', 7, 'Color', obj.rgb(2,:), 'MarkerFaceColor', obj.rgb(2,:));
                legend(ax, {'Desired path','Actual path','Start','End'}, ...
                    'Location', 'best', 'FontSize', obj.legendFontSize);
            else
                legend(ax, {'Desired path','Actual path','Start'}, ...
                    'Location', 'best', 'FontSize', obj.legendFontSize);
            end
            xlabel(ax, 'X [m]'); ylabel(ax, 'Y [m]'); zlabel(ax, 'Z [m]');
            axis(ax, 'equal');
        end

        function plotXYView(obj, logs)
            %PLOTXYVIEW Plot desired vs actual XY projection.
            %   Input:
            %     logs - logger struct.
            ax = gca; hold(ax, 'on'); obj.applyPlotStyle(ax);
            title(ax, 'XY Path');
            plot(ax, logs.des.pos(:,1), logs.des.pos(:,2), 'k--', 'LineWidth', obj.referenceLineWidth());
            plot(ax, logs.actual.pos(:,1), logs.actual.pos(:,2), '-', ...
                'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
            xlabel(ax, 'X [m]'); ylabel(ax, 'Y [m]');
            legend(ax, {'Desired path','Actual path'}, 'Location', 'best', 'FontSize', obj.legendFontSize);
            axis(ax, 'equal');
        end

        function plotZvsT(obj, logs, varargin)
            %PLOTZVST Plot altitude over time.
            %   Input:
            %     logs - logger struct.
            %     est - estimation struct (optional).
            est = [];
            if ~isempty(varargin)
                est = varargin{1};
            end
            ax = gca; hold(ax, 'on'); obj.applyPlotStyle(ax);
            title(ax, 'Altitude');
            plot(ax, logs.t, logs.actual.pos(:,3), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
            plot(ax, logs.t, logs.des.pos(:,3), '--', 'LineWidth', obj.referenceLineWidth(), 'Color', obj.rgb(1,:));
            xlabel(ax, 'Time [s]'); ylabel(ax, 'Z [m]');
            legend(ax, {'Actual z','Desired z'}, 'Location', 'best', 'FontSize', obj.legendFontSize);
            obj.setXLim(ax, logs.t);
            if isstruct(est) && isfield(est, 'dropTime')
                xline(ax, est.dropTime, 'r:', 'LineWidth', 1.5, 'HandleVisibility', 'off');
            end
        end

        function plotPosition(obj, logs, varargin)
            %PLOTPOSITION Plot position components over time.
            %   Input:
            %     logs - logger struct.
            %     est - estimation struct (optional).
            est = [];
            if ~isempty(varargin)
                est = varargin{1};
            end
            ax = gca; hold(ax, 'on'); obj.applyPlotStyle(ax);
            title(ax, 'Position [m]');
            plot(ax, logs.t, logs.actual.pos(:,1), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
            plot(ax, logs.t, logs.actual.pos(:,2), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(2,:));
            plot(ax, logs.t, logs.actual.pos(:,3), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(3,:));
            plot(ax, logs.t, logs.des.pos(:,1), '--', 'LineWidth', obj.referenceLineWidth(), 'Color', obj.rgb(1,:));
            plot(ax, logs.t, logs.des.pos(:,2), '--', 'LineWidth', obj.referenceLineWidth(), 'Color', obj.rgb(2,:));
            plot(ax, logs.t, logs.des.pos(:,3), '--', 'LineWidth', obj.referenceLineWidth(), 'Color', obj.rgb(3,:));
            legend(ax, {'$x$','$y$','$z$','$x_d$','$y_d$','$z_d$'}, ...
                'Location', 'best', 'FontSize', obj.legendFontSize, 'Interpreter', 'latex');
            set(ax, 'XTickLabel', []);
            obj.setXLim(ax, logs.t);
            if isstruct(est) && isfield(est, 'dropTime')
                xline(ax, est.dropTime, 'r:', 'LineWidth', 1.5, 'HandleVisibility', 'off');
            end
        end

        function plotOrientation(obj, logs, varargin)
            %PLOTORIENTATION Plot roll/pitch/yaw over time.
            %   Input:
            %     logs - logger struct.
            %     est - estimation struct (optional).
            est = [];
            if ~isempty(varargin)
                est = varargin{1};
            end
            ax = gca; hold(ax, 'on'); obj.applyPlotStyle(ax);
            title(ax, 'Orientation [deg]');
            rpy_act = logs.actual.rpy * (180/pi);
            rpy_des = logs.des.rpy * (180/pi);
            plot(ax, logs.t, rpy_act(:,1), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
            plot(ax, logs.t, rpy_act(:,2), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(2,:));
            plot(ax, logs.t, rpy_act(:,3), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(3,:));
            plot(ax, logs.t, rpy_des(:,1), '--', 'LineWidth', obj.referenceLineWidth(), 'Color', obj.rgb(1,:));
            plot(ax, logs.t, rpy_des(:,2), '--', 'LineWidth', obj.referenceLineWidth(), 'Color', obj.rgb(2,:));
            plot(ax, logs.t, rpy_des(:,3), '--', 'LineWidth', obj.referenceLineWidth(), 'Color', obj.rgb(3,:));
            legend(ax, {'$\phi$','$\theta$','$\psi$','$\phi_d$','$\theta_d$','$\psi_d$'}, ...
                'Location', 'best', 'FontSize', obj.legendFontSize, 'Interpreter', 'latex');
            xlabel(ax, 'Time [s]');
            obj.setXLim(ax, logs.t);
            if isstruct(est) && isfield(est, 'dropTime')
                xline(ax, est.dropTime, 'r:', 'LineWidth', 1.5, 'HandleVisibility', 'off');
            end
        end

        function plotLinearVel(obj, logs, varargin)
            %PLOTLINEARVEL Plot linear velocity components.
            %   Input:
            %     logs - logger struct.
            %     est - estimation struct (optional).
            est = [];
            if ~isempty(varargin)
                est = varargin{1};
            end
            ax = gca; hold(ax, 'on'); obj.applyPlotStyle(ax);
            title(ax, 'Linear Vel [m/s]');
            plot(ax, logs.t, logs.actual.linVel(:,1), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
            plot(ax, logs.t, logs.actual.linVel(:,2), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(2,:));
            plot(ax, logs.t, logs.actual.linVel(:,3), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(3,:));
            plot(ax, logs.t, logs.des.linVel(:,1), '--', 'LineWidth', obj.referenceLineWidth(), 'Color', obj.rgb(1,:));
            plot(ax, logs.t, logs.des.linVel(:,2), '--', 'LineWidth', obj.referenceLineWidth(), 'Color', obj.rgb(2,:));
            plot(ax, logs.t, logs.des.linVel(:,3), '--', 'LineWidth', obj.referenceLineWidth(), 'Color', obj.rgb(3,:));
            legend(ax, {'$v_x$','$v_y$','$v_z$','$v_{x,d}$','$v_{y,d}$','$v_{z,d}$'}, ...
                'Location', 'best', 'FontSize', obj.legendFontSize, 'Interpreter', 'latex');
            set(ax, 'XTickLabel', []);
            obj.setXLim(ax, logs.t);
            if isstruct(est) && isfield(est, 'dropTime')
                xline(ax, est.dropTime, 'r:', 'LineWidth', 1.5, 'HandleVisibility', 'off');
            end
        end

        function plotAngularVel(obj, logs, varargin)
            %PLOTANGULARVEL Plot angular velocity components.
            %   Input:
            %     logs - logger struct.
            %     est - estimation struct (optional).
            est = [];
            if ~isempty(varargin)
                est = varargin{1};
            end
            ax = gca; hold(ax, 'on'); obj.applyPlotStyle(ax);
            title(ax, 'Angular Vel [rad/s]');
            plot(ax, logs.t, logs.actual.angVel(:,1), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
            plot(ax, logs.t, logs.actual.angVel(:,2), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(2,:));
            plot(ax, logs.t, logs.actual.angVel(:,3), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(3,:));
            plot(ax, logs.t, logs.des.angVel(:,1), '--', 'LineWidth', obj.referenceLineWidth(), 'Color', obj.rgb(1,:));
            plot(ax, logs.t, logs.des.angVel(:,2), '--', 'LineWidth', obj.referenceLineWidth(), 'Color', obj.rgb(2,:));
            plot(ax, logs.t, logs.des.angVel(:,3), '--', 'LineWidth', obj.referenceLineWidth(), 'Color', obj.rgb(3,:));
            legend(ax, {'$\omega_x$','$\omega_y$','$\omega_z$','$\omega_{x,d}$','$\omega_{y,d}$','$\omega_{z,d}$'}, ...
                'Location', 'best', 'FontSize', obj.legendFontSize, 'Interpreter', 'latex');
            xlabel(ax, 'Time [s]');
            obj.setXLim(ax, logs.t);
            if isstruct(est) && isfield(est, 'dropTime')
                xline(ax, est.dropTime, 'r:', 'LineWidth', 1.5, 'HandleVisibility', 'off');
            end
        end

        function plotForce(obj, logs, varargin)
            %PLOTFORCE Plot commanded forces over time.
            %   Input:
            %     logs - logger struct.
            %     est - estimation struct (optional).
            est = [];
            if ~isempty(varargin)
                est = varargin{1};
            end
            ax = gca; hold(ax, 'on'); obj.applyPlotStyle(ax);
            title(ax, 'Force [N]');
            plot(ax, logs.t, logs.cmd.wrenchF(:,1), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
            plot(ax, logs.t, logs.cmd.wrenchF(:,2), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(2,:));
            plot(ax, logs.t, logs.cmd.wrenchF(:,3), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(3,:));
            legend(ax, {'$F_x$','$F_y$','$F_z$'}, ...
                'Location', 'best', 'FontSize', obj.legendFontSize, 'Interpreter', 'latex');
            set(ax, 'XTickLabel', []);
            obj.setXLim(ax, logs.t);
            if isstruct(est) && isfield(est, 'dropTime')
                xline(ax, est.dropTime, 'r:', 'LineWidth', 1.5, 'HandleVisibility', 'off');
            end
        end

        function plotTorque(obj, logs, varargin)
            %PLOTTORQUE Plot commanded torques over time.
            %   Input:
            %     logs - logger struct.
            %     est - estimation struct (optional).
            est = [];
            if ~isempty(varargin)
                est = varargin{1};
            end
            ax = gca; hold(ax, 'on'); obj.applyPlotStyle(ax);
            title(ax, 'Torque [Nm]');
            plot(ax, logs.t, logs.cmd.wrenchT(:,1), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
            plot(ax, logs.t, logs.cmd.wrenchT(:,2), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(2,:));
            plot(ax, logs.t, logs.cmd.wrenchT(:,3), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(3,:));
            legend(ax, {'$\tau_x$','$\tau_y$','$\tau_z$'}, ...
                'Location', 'best', 'FontSize', obj.legendFontSize, 'Interpreter', 'latex');
            xlabel(ax, 'Time [s]');
            obj.setXLim(ax, logs.t);
            if isstruct(est) && isfield(est, 'dropTime')
                xline(ax, est.dropTime, 'r:', 'LineWidth', 1.5, 'HandleVisibility', 'off');
            end
        end

        function plotMass(obj, est)
            %PLOTMASS Plot estimated mass over time.
            %   Input:
            %     est - estimation struct.
            ax = gca; hold(ax, 'on'); obj.applyPlotStyle(ax);
            title(ax, 'Mass [kg]');
            plot(ax, est.t, est.mass, '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
            if isfield(est, 'massActual')
                plot(ax, est.t, est.massActual, '--', 'LineWidth', obj.referenceLineWidth(), 'Color', obj.rgb(1,:));
                legend(ax, {'Estimated $\hat{m}$','True $m$'}, ...
                    'Location', 'best', 'FontSize', obj.legendFontSize, 'Interpreter', 'latex');
            else
                legend(ax, {'Estimated $\hat{m}$'}, ...
                    'Location', 'best', 'FontSize', obj.legendFontSize, 'Interpreter', 'latex');
            end
            set(ax, 'XTickLabel', []);
            obj.setXLim(ax, est.t);
            if isfield(est, 'dropTime')
                xline(ax, est.dropTime, 'r:', 'LineWidth', 1.5, 'HandleVisibility', 'off');
            end
        end

        function plotCoG(obj, est)
            %PLOTCOG Plot estimated CoG components.
            %   Input:
            %     est - estimation struct.
            ax = gca; hold(ax, 'on'); obj.applyPlotStyle(ax);
            title(ax, 'CoG [m]');
            plot(ax, est.t, est.com(:,1), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
            plot(ax, est.t, est.com(:,2), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(2,:));
            plot(ax, est.t, est.com(:,3), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(3,:));
            if isfield(est, 'comActual')
                plot(ax, est.t, est.comActual(:,1), '--', 'LineWidth', obj.referenceLineWidth(), 'Color', obj.rgb(1,:));
                plot(ax, est.t, est.comActual(:,2), '--', 'LineWidth', obj.referenceLineWidth(), 'Color', obj.rgb(2,:));
                plot(ax, est.t, est.comActual(:,3), '--', 'LineWidth', obj.referenceLineWidth(), 'Color', obj.rgb(3,:));
                legend(ax, {'$\hat{\xi}_x$','$\hat{\xi}_y$','$\hat{\xi}_z$', ...
                            '$\xi_x$','$\xi_y$','$\xi_z$'}, ...
                    'Location', 'best', 'FontSize', obj.legendFontSize, 'Interpreter', 'latex');
            else
                legend(ax, {'$\hat{\xi}_x$','$\hat{\xi}_y$','$\hat{\xi}_z$'}, ...
                    'Location', 'best', 'FontSize', obj.legendFontSize, 'Interpreter', 'latex');
            end
            xlabel(ax, 'Time [s]');
            obj.setXLim(ax, est.t);
            if isfield(est, 'dropTime')
                xline(ax, est.dropTime, 'r:', 'LineWidth', 1.5, 'HandleVisibility', 'off');
            end
        end

        function plotPrincipalInertia(obj, est)
            %PLOTPRINCIPALINERTIA Plot principal inertia estimates.
            %   Input:
            %     est - estimation struct.
            ax = gca; hold(ax, 'on'); obj.applyPlotStyle(ax);
            title(ax, 'Principal Inertia [kg·m²]');
            plot(ax, est.t, est.inertia(:,1), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
            plot(ax, est.t, est.inertia(:,2), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(2,:));
            plot(ax, est.t, est.inertia(:,3), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(3,:));
            if isfield(est, 'inertiaActual')
                plot(ax, est.t, est.inertiaActual(:,1), '--', 'LineWidth', obj.referenceLineWidth(), 'Color', obj.rgb(1,:));
                plot(ax, est.t, est.inertiaActual(:,2), '--', 'LineWidth', obj.referenceLineWidth(), 'Color', obj.rgb(2,:));
                plot(ax, est.t, est.inertiaActual(:,3), '--', 'LineWidth', obj.referenceLineWidth(), 'Color', obj.rgb(3,:));
                legend(ax, {'$\hat{I}_{xx}$','$\hat{I}_{yy}$','$\hat{I}_{zz}$', ...
                            '$I_{xx}$','$I_{yy}$','$I_{zz}$'}, ...
                    'Location', 'best', 'FontSize', obj.legendFontSize, 'Interpreter', 'latex');
            else
                legend(ax, {'$\hat{I}_{xx}$','$\hat{I}_{yy}$','$\hat{I}_{zz}$'}, ...
                    'Location', 'best', 'FontSize', obj.legendFontSize, 'Interpreter', 'latex');
            end
            xlabel(ax, 'Time [s]');
            obj.setXLim(ax, est.t);
            if isfield(est, 'dropTime')
                xline(ax, est.dropTime, 'r:', 'LineWidth', 1.5, 'HandleVisibility', 'off');
            end
        end

        function plotOffDiagInertia(obj, est)
            %PLOTOFFDIAGINERTIA Plot off-diagonal inertia estimates.
            %   Input:
            %     est - estimation struct.
            ax = gca; hold(ax, 'on'); obj.applyPlotStyle(ax);
            title(ax, 'Off-diag Inertia [kg·m²]');
            if size(est.inertia, 2) >= 6
                plot(ax, est.t, est.inertia(:,4), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(1,:));
                plot(ax, est.t, est.inertia(:,5), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(2,:));
                plot(ax, est.t, est.inertia(:,6), '-', 'LineWidth', obj.lineWidth, 'Color', obj.rgb(3,:));
                if isfield(est, 'inertiaActual') && size(est.inertiaActual, 2) >= 6
                    plot(ax, est.t, est.inertiaActual(:,4), '--', 'LineWidth', obj.referenceLineWidth(), 'Color', obj.rgb(1,:));
                    plot(ax, est.t, est.inertiaActual(:,5), '--', 'LineWidth', obj.referenceLineWidth(), 'Color', obj.rgb(2,:));
                    plot(ax, est.t, est.inertiaActual(:,6), '--', 'LineWidth', obj.referenceLineWidth(), 'Color', obj.rgb(3,:));
                    legend(ax, {'$\hat{I}_{xy}$','$\hat{I}_{yz}$','$\hat{I}_{zx}$', ...
                                '$I_{xy}$','$I_{yz}$','$I_{zx}$'}, ...
                        'Location', 'best', 'FontSize', obj.legendFontSize, 'Interpreter', 'latex');
                else
                    legend(ax, {'$\hat{I}_{xy}$','$\hat{I}_{yz}$','$\hat{I}_{zx}$'}, ...
                        'Location', 'best', 'FontSize', obj.legendFontSize, 'Interpreter', 'latex');
                end
            end
            xlabel(ax, 'Time [s]');
            obj.setXLim(ax, est.t);
            if isfield(est, 'dropTime')
                xline(ax, est.dropTime, 'r:', 'LineWidth', 1.5, 'HandleVisibility', 'off');
            end
        end

        function setXLim(obj, ax, tVec)
            %SETXLIM Apply time-axis limits.
            %   Inputs:
            %     ax - axes handle.
            %     tVec - time vector.
            if ~isempty(obj.duration)
                xlim(ax, [0 obj.duration]);
            else
                xlim(ax, [0 max(tVec)]);
            end
        end

        function applyPlotStyle(obj, ax)
            %APPLYPLOTSTYLE Apply consistent styling to axes.
            %   Input:
            %     ax - axes handle.
            box(ax, 'on');
            grid(ax, 'on');
            ax.GridAlpha = 0.25;
            ax.GridLineStyle = ':';
            ax.BoxStyle = 'full';
            ax.LineWidth = 1.2;
            set(ax, 'FontSize', obj.labelFontSize);
            ax.Title.FontSize = obj.titleFontSize;
            ax.XLabel.FontSize = obj.labelFontSize;
            ax.YLabel.FontSize = obj.labelFontSize;
            ax.ZLabel.FontSize = obj.labelFontSize;
        end

        function saveFigureInternal(obj, fig, filename)
            %SAVEFIGUREINTERNAL Save figure to output directory.
            %   Inputs:
            %     fig - figure handle.
            %     filename - base file name.
            if obj.savePng
                if ~isempty(obj.outDir) && ~exist(obj.outDir, 'dir')
                    mkdir(obj.outDir);
                end
                filepath = fullfile(obj.outDir, [filename '.png']);
                set(fig, 'Color', 'w');
                set(fig, 'PaperPositionMode', 'auto');
                drawnow;
                obj.padAxesForExport(fig);
                drawnow;
                try
                    print(fig, filepath, '-dpng', '-r300');
                catch
                    try
                        saveas(fig, filepath);
                    catch
                        exportgraphics(fig, filepath, 'Resolution', 300);
                    end
                end
            end
        end

        function w = referenceLineWidth(obj)
            %REFERENCELINEWIDTH Use a lighter width for reference signals.
            w = max(1.0, obj.lineWidth - 0.5);
        end

        function ax = createAxesInPosition(~, fig, pos, nRows, m, gv)
            %CREATEAXESINPOSITION Create stacked axes within a parent position.
            if nargin < 5 || isempty(m)
                m = [0.02 0.05 0.02 0.06];
            end
            if nargin < 6 || isempty(gv)
                gv = 0.04;
            end

            validateattributes(pos, {'numeric'}, {'vector','numel',4});
            validateattributes(m, {'numeric'}, {'vector','numel',4,'>=',0,'<=',0.5});
            validateattributes(gv, {'numeric'}, {'scalar','>=',0,'<=',0.5});

            inner = [
                pos(1) + pos(3) * m(4), ...
                pos(2) + pos(4) * m(2), ...
                pos(3) * (1 - m(3) - m(4)), ...
                pos(4) * (1 - m(1) - m(2))
            ];
            rowGap = pos(4) * gv;
            h = (inner(4) - (nRows - 1) * rowGap) / nRows;

            ax = gobjects(nRows, 1);
            for i = 1:nRows
                y = inner(2) + inner(4) - i * h - (i - 1) * rowGap;
                ax(i) = axes('Parent', fig, 'Position', [inner(1), y, inner(3), h]);
                box(ax(i), 'on');
                ax(i).BoxStyle = 'full';
                ax(i).LineWidth = 1.2;
            end
        end

        function padAxesForExport(~, fig)
            %PADAXESFOREXPORT Keep labels and ticks inside figure bounds.
            ax = findall(fig, 'Type', 'axes');
            if isempty(ax)
                return;
            end

            outerPad = [0.02 0.02 0.01 0.01]; % [left bottom right top]
            for i = 1:numel(ax)
                if ~isgraphics(ax(i), 'axes') || strcmp(get(ax(i), 'Visible'), 'off')
                    continue;
                end

                pos = get(ax(i), 'Position');
                ti = get(ax(i), 'TightInset');
                if numel(pos) ~= 4 || numel(ti) ~= 4
                    continue;
                end

                left = pos(1) - ti(1);
                bottom = pos(2) - ti(2);
                right = pos(1) + pos(3) + ti(3);
                top = pos(2) + pos(4) + ti(4);

                dx = 0;
                dy = 0;
                shrinkW = 0;
                shrinkH = 0;

                if left < outerPad(1)
                    dx = outerPad(1) - left;
                end
                if bottom < outerPad(2)
                    dy = outerPad(2) - bottom;
                end
                if right > 1 - outerPad(3)
                    shrinkW = right - (1 - outerPad(3));
                end
                if top > 1 - outerPad(4)
                    shrinkH = top - (1 - outerPad(4));
                end

                pos(1) = min(max(pos(1) + dx, outerPad(1)), 1 - outerPad(3));
                pos(2) = min(max(pos(2) + dy, outerPad(2)), 1 - outerPad(4));
                pos(3) = max(0.05, pos(3) - shrinkW);
                pos(4) = max(0.05, pos(4) - shrinkH);

                if pos(1) + pos(3) > 1 - outerPad(3)
                    pos(3) = max(0.05, 1 - outerPad(3) - pos(1));
                end
                if pos(2) + pos(4) > 1 - outerPad(4)
                    pos(4) = max(0.05, 1 - outerPad(4) - pos(2));
                end

                set(ax(i), 'Position', pos);
            end
        end

        function pos = mergePositions(~, topPos, bottomPos)
            %MERGEPOSITIONS Merge aligned positions into one enclosing box.
            left = min(topPos(1), bottomPos(1));
            bottom = min(topPos(2), bottomPos(2));
            right = max(topPos(1) + topPos(3), bottomPos(1) + bottomPos(3));
            top = max(topPos(2) + topPos(4), bottomPos(2) + bottomPos(4));
            pos = [left, bottom, right - left, top - bottom];
        end

        function posGrid = buildWeightedGridPositions(~, rowWeights, colWeights, m, g)
            %BUILDWEIGHTEDGRIDPOSITIONS Build positions with weighted rows/cols.
            validateattributes(rowWeights, {'numeric'}, {'vector','nonempty','positive'});
            validateattributes(colWeights, {'numeric'}, {'vector','nonempty','positive'});
            validateattributes(m, {'numeric'}, {'vector','numel',4,'>=',0,'<=',0.5});
            validateattributes(g, {'numeric'}, {'vector','numel',2,'>=',0,'<=',0.5});

            mt = m(1);
            mb = m(2);
            mr = m(3);
            ml = m(4);
            gv = g(1);
            gh = g(2);
            rows = numel(rowWeights);
            cols = numel(colWeights);

            totalW = 1 - ml - mr - (cols - 1) * gh;
            totalH = 1 - mt - mb - (rows - 1) * gv;
            colWidths = totalW * (colWeights(:)' / sum(colWeights));
            rowHeights = totalH * (rowWeights(:)' / sum(rowWeights));

            posGrid = cell(rows, cols);
            yTop = 1 - mt;
            for r = 1:rows
                yTop = yTop - rowHeights(r);
                x = ml;
                for c = 1:cols
                    posGrid{r, c} = [x, yTop, colWidths(c), rowHeights(r)];
                    x = x + colWidths(c) + gh;
                end
                yTop = yTop - gv;
            end
        end
    end
end
