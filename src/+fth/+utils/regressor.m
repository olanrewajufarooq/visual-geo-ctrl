function Y = regressor(H, VR, VRDot, g, coriolisFactor)
%REGRESSOR Rigid-body dynamics regressor consistent with factorization.
%   Y * pi = I6(pi) * VRDot + C_factor(VR, I6(pi)) * VR + Wg(pi)
%
%   Parameter convention:
%     pi = [m; hx; hy; hz; Ixx; Iyy; Izz; Ixy; Ixz; Iyz]
%
%   Inputs:
%     H             - 4x4 pose (used for gravity direction)
%     VR            - 6x1 reference body velocity
%     VRDot         - 6x1 reference body acceleration
%     g             - scalar gravity magnitude
%     coriolisFactor - CoriolisFactorBase instance
%
%   Output:
%     Y - 6x10 regressor matrix

    if nargin < 5 || isempty(coriolisFactor)
        coriolisFactor = fth.ctrl.coriolis.basicCoriolisFactor();
    end
    if nargin < 4 || isempty(g)
        g = 9.81;
    end

    R = H(1:3,1:3);
    gvec = [0; 0; g];
    g_body = R' * gvec;

    Y = zeros(6, 10, 'like', VR);

    for i = 1:10
        pi_i = zeros(10, 1, 'like', VR);
        pi_i(i) = 1;

        I6_i = fth.utils.RBInertia.params2genInertia(pi_i);
        C_i = coriolisFactor.getCoriolisFactor(VR, I6_i);

        m_i = pi_i(1);
        h_i = pi_i(2:4);

        Wg_i = [cross(h_i, g_body); m_i * g_body];

        Y(:, i) = I6_i * VRDot + C_i * VR + Wg_i;
    end
end