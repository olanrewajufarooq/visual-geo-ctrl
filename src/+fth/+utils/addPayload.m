function [m_total, Iparams_total, cog_total] = addPayload(m_base, Iparams_base, cog_base, m_payload, cog_payload)
%ADDPAYLOAD Combine payload mass/CoG with base parameters.
%   CoG: 3x1 offset of center of mass from body frame origin, in body frame [m].
    assert(isscalar(m_base) && m_base > 0, 'fth:addPayload: m_base must be a positive scalar');
    assert(numel(Iparams_base) == 6, 'fth:addPayload: Iparams_base must have 6 elements');
    assert(numel(cog_base) == 3, 'fth:addPayload: cog_base must be a 3-element vector');
    assert(isscalar(m_payload) && m_payload >= 0, 'fth:addPayload: m_payload must be a non-negative scalar');
    assert(numel(cog_payload) == 3, 'fth:addPayload: cog_payload must be a 3-element vector');

    m_total = m_base + m_payload;
    cog_total = (m_base * cog_base + m_payload * cog_payload(:)) / m_total;

    I_base = fth.utils.inertiaFromParams(Iparams_base);
    % Payload treated as a point mass (no self-inertia); Steiner term only.
    r = cog_payload(:);
    I_payload = m_payload * ((dot(r,r) * eye(3)) - (r * r.'));
    I_total = I_base + I_payload;

    % Pack in declared order: [Ixx Iyy Izz Ixy Iyz Ixz].
    Iparams_total = [I_total(1,1), I_total(2,2), I_total(3,3), ...
                     I_total(1,2), I_total(2,3), I_total(1,3)];
end
