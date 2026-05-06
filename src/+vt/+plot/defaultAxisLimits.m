function limits = defaultAxisLimits(cfg)
%DEFAULTAXISLIMITS Compute initial axis limits for visualization
    mode = 'auto';
    if isfield(cfg, 'viz') && isfield(cfg.viz, 'initialAxis')
        mode = cfg.viz.initialAxis;
    end

    if strcmpi(mode, 'fixed10')
        half = 5;
        limits = [-half half -half half -half half];
        return;
    end

    scale = cfg.traj.scale;
    alt = cfg.traj.altitude;
    pad = 2.0;
    if isfield(cfg, 'viz') && isfield(cfg.viz, 'axisPadding')
        pad = cfg.viz.axisPadding;
    end

    switch lower(cfg.traj.name)
        case 'circle'
            xMin = -2*scale; xMax = 0;
            yMin = -scale;   yMax = scale;
        case 'infinity'
            xMin = -scale; xMax = scale;
            yMin = -scale/2; yMax = scale/2;
        case 'infinity3d'
            xMin = -scale; xMax = scale;
            yMin = -scale/2; yMax = scale/2;
        case 'infinity3dmod'
            alpha = 0;
            if isfield(cfg.traj, 'inf3dModAlpha')
                alpha = cfg.traj.inf3dModAlpha;
            end
            xMin = -scale*(1+abs(alpha)); xMax = scale*(1+abs(alpha));
            yMin = -(scale/2)*(1+abs(alpha)); yMax = (scale/2)*(1+abs(alpha));
        case 'lissajous3d'
            amp = [scale; scale; min(scale/2, alt/2)];
            if isfield(cfg.traj, 'lissajousAmp') && ~isempty(cfg.traj.lissajousAmp)
                amp = cfg.traj.lissajousAmp(:);
                if numel(amp) == 1
                    amp = repmat(amp, 3, 1);
                end
            end
            xMin = -abs(amp(1)); xMax = abs(amp(1));
            yMin = -abs(amp(2)); yMax = abs(amp(2));
        case 'helix3d'
            xMin = -scale; xMax = scale;
            yMin = -scale; yMax = scale;
        case 'poly3d'
            xMin = -scale; xMax = scale;
            yMin = -scale; yMax = scale;
        case 'takeoffland'
            xMin = 0; xMax = scale;
            yMin = 0; yMax = 0;
        otherwise
            xMin = 0; xMax = 0;
            yMin = 0; yMax = 0;
    end

    maxXY = max(abs([xMin xMax yMin yMax]));
    maxXY = maxXY + pad;

    zMin = 0; zMax = alt;
    if any(strcmpi(cfg.traj.name, {'infinity3d'}))
        zMin = min(zMin, alt - scale);
        zMax = max(zMax, alt + scale);
    elseif any(strcmpi(cfg.traj.name, {'infinity3dmod'}))
        beta = 0;
        if isfield(cfg.traj, 'inf3dModBeta')
            beta = cfg.traj.inf3dModBeta;
        end
        z_amp = min(scale/2, alt/2) * (1 + abs(beta));
        zMin = min(zMin, alt - z_amp);
        zMax = max(zMax, alt + z_amp);
    elseif any(strcmpi(cfg.traj.name, {'lissajous3d'}))
        z_amp = min(scale/2, alt/2);
        if isfield(cfg.traj, 'lissajousAmp') && ~isempty(cfg.traj.lissajousAmp)
            amp = cfg.traj.lissajousAmp(:);
            if numel(amp) == 1
                amp = repmat(amp, 3, 1);
            end
            if numel(amp) >= 3
                z_amp = abs(amp(3));
            end
        end
        zMin = min(zMin, alt - z_amp);
        zMax = max(zMax, alt + z_amp);
    elseif any(strcmpi(cfg.traj.name, {'helix3d'}))
        z_amp = min(scale/2, alt/2);
        if isfield(cfg.traj, 'helixZAmp') && ~isempty(cfg.traj.helixZAmp)
            z_amp = abs(cfg.traj.helixZAmp);
        end
        zMin = min(zMin, alt - z_amp);
        zMax = max(zMax, alt + z_amp);
    end
    zMax = zMax + pad;
    zMax = max(zMax, 0);

    limits = [-maxXY maxXY -maxXY maxXY zMin zMax];
end
