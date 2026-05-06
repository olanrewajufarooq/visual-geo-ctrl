function AdInv = Ad_inv(T)
%AD_INV Inverse adjoint of an SE(3) element.
%   Ad_inv(H) = Ad(H^{-1}) = [R', 0; -R'*hat(p), R'].
%
%   Input:  T - 4x4 SE(3) matrix.
%   Output: AdInv - 6x6 inverse adjoint matrix.
    R = T(1:3,1:3);
    p = T(1:3,4);
    AdInv = [R', zeros(3); -R' * vt.se3.hat3(p), R'];
end
