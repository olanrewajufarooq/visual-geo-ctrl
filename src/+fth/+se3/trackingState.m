function st = trackingState(H, Hd, V, Vd)
%TRACKINGSTATE Extract full pose and velocity tracking quantities.
%   Extends poseDecompose with body-frame velocity components and the
%   classical velocity error Ve = V - Ad^{-1}(He)*Vd.
%
%   Inputs:
%     H  - 4x4 actual pose (SE(3)).
%     Hd - 4x4 desired pose (SE(3)).
%     V  - 6x1 actual body velocity [omega; v].
%     Vd - 6x1 desired body velocity [omega_d; v_d].
%
%   Output fields (includes all poseDecompose fields, plus):
%     omega   - 3x1 actual body angular velocity.
%     v       - 3x1 actual body linear velocity.
%     omega_d - 3x1 desired body angular velocity.
%     v_d     - 3x1 desired body linear velocity.
%     Ve      - 6x1 classical velocity error: V - Ad^{-1}(He)*Vd.
%     omega_e - 3x1 angular velocity error: Ve(1:3) = omega - Re^T*omega_d.

    st         = fth.se3.poseDecompose(H, Hd);
    st.omega   = V(1:3);
    st.v       = V(4:6);
    st.omega_d = Vd(1:3);
    st.v_d     = Vd(4:6);
    st.Ve      = V - st.AdInvHe * Vd;
    st.omega_e = st.Ve(1:3);
end
