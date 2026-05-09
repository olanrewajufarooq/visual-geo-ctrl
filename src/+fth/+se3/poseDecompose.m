function st = poseDecompose(H, Hd)
%POSEDECOMPOSE Extract pose-level geometric quantities from H and Hd.
%   Returns a struct with rotation matrices, position vectors, the pose
%   error transformation, and its inverse adjoint. Used by potential
%   functions to avoid repeating the same extractions.
%
%   Inputs:
%     H  - 4x4 actual pose (SE(3)).
%     Hd - 4x4 desired pose (SE(3)).
%
%   Output fields:
%     R      - 3x3 actual rotation matrix.
%     Rd     - 3x3 desired rotation matrix.
%     Re     - 3x3 rotation error: Re = Rd^T * R.
%     xi     - 3x1 actual position (world frame).
%     xi_d   - 3x1 desired position (world frame).
%     xi_e   - 3x1 position difference: xi - xi_d (world frame).
%     He     - 4x4 pose error: He = Hd^{-1} * H.
%     AdInvHe- 6x6 inverse adjoint of He.

    st.R       = H(1:3,1:3);
    st.Rd      = Hd(1:3,1:3);
    st.Re      = st.Rd' * st.R;
    st.xi      = H(1:3,4);
    st.xi_d    = Hd(1:3,4);
    st.xi_e    = st.xi - st.xi_d;
    st.He      = fth.se3.invSE3(Hd) * H;
    st.AdInvHe = fth.se3.Ad_inv(st.He);
end
