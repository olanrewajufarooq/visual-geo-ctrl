function [m_total, Iparams_total, cog_total] = addPayload(m_base, Iparams_base, cog_base, m_payload, cog_payload)
%ADDPAYLOAD Combine payload mass/CoG with base parameters
    m_total = m_base + m_payload;
    cog_total = (m_base * cog_base + m_payload * cog_payload(:)) / m_total;

    I_base = vt.utils.inertiaFromParams(Iparams_base);
    r = cog_payload(:);
    I_payload = m_payload * ((dot(r,r) * eye(3)) - (r * r.'));
    I_total = I_base + I_payload;

    Iparams_total = [I_total(1,1), I_total(2,2), I_total(3,3), I_total(1,2), I_total(1,3), I_total(2,3)];
end
