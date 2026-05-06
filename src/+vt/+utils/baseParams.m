function [m_base, Iparams_base, cog_base] = baseParams(cfg)
%BASEPARAMS Extract base parameters from config
    m_base = cfg.vehicle.m;
    Iparams_base = cfg.vehicle.I_params;
    cog_base = cfg.vehicle.CoG(:);
end
