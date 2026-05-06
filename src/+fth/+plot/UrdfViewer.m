classdef UrdfViewer < handle
    %URDFVIEWER Visualize the hexacopter using URDF or a fallback model.
    %   Uses Robotics System Toolbox when available; otherwise draws a
    %   lightweight stick model and path traces.
    %
    %   The viewer can also update desired/actual path trails.
    properties
        urdfPath
        robot
        fig
        ax
        baseTf
        hasRobotics
        hFallback
        axisLimits
        armLength
        dynamicAxis
        axisPadding
        hOrigin
        hPathDesired
        hPathActual
        pathDesired
        pathActual
    end

    methods
        function obj = UrdfViewer(urdfPath, ax, useRobotics)
            %URDFVIEWER Configure visualization and load URDF if available.
            %   Inputs:
            %     urdfPath - path to URDF file (optional).
            %     ax - axes handle to render into (optional).
            %     useRobotics - true to use Robotics System Toolbox.
            %   Output:
            %     obj - UrdfViewer instance.
            if nargin < 3 || isempty(useRobotics)
                useRobotics = true;
            end
            if nargin < 1 || isempty(urdfPath)
                urdfPath = obj.defaultUrdfPath();
            end
            obj.urdfPath = urdfPath;
            obj.hasRobotics = useRobotics && exist('importrobot','file') == 2 && exist(urdfPath,'file') == 2;

            obj.axisLimits = [-1 1 -1 1 -0.1 2];
            obj.armLength = 0.229;
            obj.dynamicAxis = false;
            obj.axisPadding = 1.5;
            obj.pathDesired = zeros(0,3);
            obj.pathActual = zeros(0,3);

            if nargin >= 2 && ~isempty(ax) && isgraphics(ax, 'axes')
                obj.ax = ax;
                obj.fig = ancestor(ax, 'figure');
            else
                obj.fig = figure('Name','URDF Viewer','Position',[100 100 800 600]);
                obj.ax = axes(obj.fig);
            end
            cla(obj.ax);
            grid(obj.ax,'on'); axis(obj.ax,'equal'); view(obj.ax,3);
            axis(obj.ax, obj.axisLimits);
            axis(obj.ax, 'manual');
            xlabel(obj.ax,'X'); ylabel(obj.ax,'Y'); zlabel(obj.ax,'Z');
            hold(obj.ax,'on');
            obj.hOrigin = plot3(obj.ax, 0, 0, 0, 'k+', 'MarkerSize', 8, 'LineWidth', 1.5);
            obj.hPathDesired = plot3(obj.ax, NaN, NaN, NaN, 'k--', 'LineWidth', 1.2);
            obj.hPathActual = plot3(obj.ax, NaN, NaN, NaN, 'b-', 'LineWidth', 1.5);

            if obj.hasRobotics
                try
                    obj.robot = importrobot(urdfPath, 'DataFormat','column');
                    obj.robot.Gravity = [0 0 -9.81];
                    cfg = homeConfiguration(obj.robot);
                    h = show(obj.robot, cfg, 'Parent', obj.ax, 'PreservePlot', false, 'Frames','off');
                    obj.baseTf = hgtransform('Parent', obj.ax);
                    set(h, 'Parent', obj.baseTf);
                    axis(obj.ax, obj.axisLimits);
                    axis(obj.ax, 'manual');
                catch err
                    try
                        urdfFixed = fth.plot.UrdfViewer.sanitizeUrdf(urdfPath);
                        obj.robot = importrobot(urdfFixed, 'DataFormat','column');
                        obj.robot.Gravity = [0 0 -9.81];
                        cfg = homeConfiguration(obj.robot);
                        h = show(obj.robot, cfg, 'Parent', obj.ax, 'PreservePlot', false, 'Frames','off');
                        obj.baseTf = hgtransform('Parent', obj.ax);
                        set(h, 'Parent', obj.baseTf);
                        axis(obj.ax, obj.axisLimits);
                        axis(obj.ax, 'manual');
                    catch err2
                        warning('URDF import failed (%s). Using fallback visualization.', '%s', err2.message);
                        obj.hasRobotics = false;
                        obj.hFallback = obj.drawFallbackModel(eye(4));
                    end
                end
            else
                obj.hFallback = obj.drawFallbackModel(eye(4));
            end
        end

        function showPose(obj, H)
            %SHOWPOSE Update the displayed pose.
            %   Input:
            %     H - 4x4 pose matrix.
            if ~isgraphics(obj.ax)
                return;
            end
            if obj.hasRobotics
                if isgraphics(obj.baseTf)
                    obj.baseTf.Matrix = H;
                end
            else
                obj.updateFallbackModel(H);
            end
            if obj.dynamicAxis
                obj.updateAxis(H(1:3,4));
            else
                axis(obj.ax, obj.axisLimits);
                axis(obj.ax, 'manual');
            end
        end

        function updatePaths(obj, pDesired, pActual)
            %UPDATEPATHS Append desired and actual path traces.
            %   Inputs:
            %     pDesired - 3x1 desired position (optional).
            %     pActual - 3x1 actual position (optional).
            if ~isgraphics(obj.ax)
                return;
            end
            if nargin >= 2 && ~isempty(pDesired)
                obj.pathDesired(end+1,:) = pDesired(:).';
                if ~isgraphics(obj.hPathDesired)
                    obj.hPathDesired = plot3(obj.ax, NaN, NaN, NaN, 'k--', 'LineWidth', 1.2);
                end
                set(obj.hPathDesired, 'XData', obj.pathDesired(:,1), 'YData', obj.pathDesired(:,2), 'ZData', obj.pathDesired(:,3));
            end
            if nargin >= 3 && ~isempty(pActual)
                obj.pathActual(end+1,:) = pActual(:).';
                if ~isgraphics(obj.hPathActual)
                    obj.hPathActual = plot3(obj.ax, NaN, NaN, NaN, 'b-', 'LineWidth', 1.5);
                end
                set(obj.hPathActual, 'XData', obj.pathActual(:,1), 'YData', obj.pathActual(:,2), 'ZData', obj.pathActual(:,3));
            end
        end

        function setAxisLimits(obj, limits)
            %SETAXISLIMITS Set fixed axis limits.
            %   Input:
            %     limits - 1x6 axis limits [xmin xmax ymin ymax zmin zmax].
            obj.axisLimits = limits(:).';
            axis(obj.ax, obj.axisLimits);
            axis(obj.ax, 'manual');
        end

        function setDynamicAxis(obj, enable, padding)
            %SETDYNAMICAXIS Enable auto-scaling with padding.
            %   Inputs:
            %     enable - true/false.
            %     padding - scalar padding for axis limits (optional).
            obj.dynamicAxis = logical(enable);
            if nargin >= 3 && ~isempty(padding)
                obj.axisPadding = padding;
            end
        end
    end

    methods (Access = private)
        function urdfPath = defaultUrdfPath(obj)
            %DEFAULTURDFPATH Resolve URDF path in assets.
            root = obj.repoRoot();
            urdfPath = fullfile(root, 'assets', 'hexacopter_description', 'urdf', 'variable_tilt_hexacopter.urdf');
            if ~exist(urdfPath,'file')
                alt = fullfile(root, 'assets', 'hexacopter_description', 'urdf', 'variable_tilt_hexacopter_adaptive.urdf');
                if exist(alt,'file')
                    urdfPath = alt;
                end
            end
        end

        function root = repoRoot(~)
            %REPOROOT Return repository root path.
            p = mfilename('fullpath');
            root = fileparts(fileparts(fileparts(fileparts(p))));
        end

        function h = drawFallbackModel(obj, H)
            %DRAWFALLBACKMODEL Draw a simple hexacopter skeleton.
            %   Input:
            %     H - 4x4 pose matrix.
            %   Output:
            %     h - struct of graphics handles.
            p = H(1:3,4);
            R = H(1:3,1:3);
            angles = (0:5) * (pi/3);
            ends = [obj.armLength * cos(angles); obj.armLength * sin(angles); zeros(1,6)];
            endsW = R * ends + p;

            hold(obj.ax,'on');
            h.center = plot3(obj.ax, p(1), p(2), p(3), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k');
            h.arms = gobjects(6,1);
            for i = 1:6
                h.arms(i) = plot3(obj.ax, [p(1) endsW(1,i)], [p(2) endsW(2,i)], [p(3) endsW(3,i)], 'b-', 'LineWidth', 2);
            end
        end

        function updateFallbackModel(obj, H)
            %UPDATEFALLBACKMODEL Update fallback model pose.
            %   Input:
            %     H - 4x4 pose matrix.
            if isempty(obj.hFallback) || ~isfield(obj.hFallback, 'center') || ~isgraphics(obj.hFallback.center)
                if ~isgraphics(obj.ax)
                    return;
                end
                obj.hFallback = obj.drawFallbackModel(H);
                return;
            end
            p = H(1:3,4);
            R = H(1:3,1:3);
            angles = (0:5) * (pi/3);
            ends = [obj.armLength * cos(angles); obj.armLength * sin(angles); zeros(1,6)];
            endsW = R * ends + p;

            set(obj.hFallback.center, 'XData', p(1), 'YData', p(2), 'ZData', p(3));
            for i = 1:6
                set(obj.hFallback.arms(i), 'XData', [p(1) endsW(1,i)], 'YData', [p(2) endsW(2,i)], 'ZData', [p(3) endsW(3,i)]);
            end
        end

        function updateAxis(obj, p)
            %UPDATEAXIS Expand axis limits to include position.
            %   Input:
            %     p - 3x1 position.
            pad = obj.axisPadding;
            lim = obj.axisLimits;

            halfXY = max(abs(lim(1:4)));
            if abs(p(1)) > halfXY || abs(p(2)) > halfXY
                newHalf = max(abs(p(1)), abs(p(2))) + pad;
                lim(1) = -newHalf;
                lim(2) =  newHalf;
                lim(3) = -newHalf;
                lim(4) =  newHalf;
            end

            if p(3) > lim(6)
                lim(6) = p(3) + pad;
            end

            lim(5) = 0;
            lim(6) = max(lim(6), 0);

            obj.axisLimits = lim;
            axis(obj.ax, obj.axisLimits);
            axis(obj.ax, 'manual');
        end

    end

    methods (Static)
        function outPath = sanitizeUrdf(inPath)
            %SANITIZEURDF Strip Gazebo/SDF extensions for MATLAB importrobot.
            %   Removes <sensor>, <plugin>, <pose>, <gravity>,
            %   <velocity_decay>, SDF-style material children, and visual
            %   name attributes. Converts nested geometry to attribute form.
            %
            %   Input:
            %     inPath - URDF file path.
            %   Output:
            %     outPath - sanitized URDF temp file path.
            txt = fileread(inPath);

            % Strip <sensor ...>...</sensor> blocks (multi-line, non-greedy).
            txt = regexprep(txt, '\s*<sensor\b[^>]*>.*?</sensor>', '', 'dotall');

            % Strip <plugin ...>...</plugin> blocks (multi-line, non-greedy).
            txt = regexprep(txt, '\s*<plugin\b[^>]*>.*?</plugin>', '', 'dotall');

            % Strip <pose>...</pose> tags (single-line).
            txt = regexprep(txt, '\s*<pose[^>]*>[^<]*</pose>', '');

            % Strip <gravity>...</gravity> and self-closing <gravity/>.
            txt = regexprep(txt, '\s*<gravity[^>]*>[^<]*</gravity>', '');
            txt = regexprep(txt, '\s*<gravity\s*/>', '');

            % Strip <velocity_decay/> and <velocity_decay>...</velocity_decay>.
            txt = regexprep(txt, '\s*<velocity_decay\s*/>', '');
            txt = regexprep(txt, '\s*<velocity_decay[^>]*>[^<]*</velocity_decay>', '');

            % Strip SDF-style material children: <ambient>, <diffuse>,
            % <specular>, <emissive>.
            txt = regexprep(txt, '\s*<ambient>[^<]*</ambient>', '');
            txt = regexprep(txt, '\s*<diffuse>[^<]*</diffuse>', '');
            txt = regexprep(txt, '\s*<specular>[^<]*</specular>', '');
            txt = regexprep(txt, '\s*<emissive>[^<]*</emissive>', '');

            % Remove name attribute from <visual> tags (not valid URDF).
            txt = regexprep(txt, '<visual\s+name="[^"]*">', '<visual>');

            % Convert nested geometry to attribute form.
            txt = regexprep(txt, '<box>\s*<size>([^<]+)</size>\s*</box>', '<box size="$1"/>');
            txt = regexprep(txt, '<cylinder>\s*<radius>([^<]+)</radius>\s*<length>([^<]+)</length>\s*</cylinder>', '<cylinder radius="$1" length="$2"/>');
            txt = regexprep(txt, '<sphere>\s*<radius>([^<]+)</radius>\s*</sphere>', '<sphere radius="$1"/>');

            % Convert nested mass to attribute form.
            txt = regexprep(txt, '<mass>\s*([^<]+)\s*</mass>', '<mass value="$1"/>');

            % Convert nested inertia to attribute form.
            txt = regexprep(txt, '<inertia>\s*<ixx>([^<]+)</ixx>\s*<ixy>([^<]+)</ixy>\s*<ixz>([^<]+)</ixz>\s*<iyy>([^<]+)</iyy>\s*<iyz>([^<]+)</iyz>\s*<izz>([^<]+)</izz>\s*</inertia>', ...
                '<inertia ixx="$1" ixy="$2" ixz="$3" iyy="$4" iyz="$5" izz="$6"/>');

            % Fix empty <material> blocks (no name attribute, children
            % already stripped above) into self-closing with default name.
            txt = regexprep(txt, '<material>\s*</material>', '<material name="default"/>');

            % Add name to remaining <material> open tags that lack one.
            txt = regexprep(txt, '<material>(?!\s*<color)', '<material name="default">');

            outPath = fullfile(tempdir, 'vt_hexacopter_sanitized.urdf');
            fid = fopen(outPath, 'w');
            fwrite(fid, txt);
            fclose(fid);
        end
    end
end
