function AdT = Ad(T)
%AD Adjoint representation of an SE(3) element.
%   Ad(H) = [R, 0; hat(p)*R, R]  where H = [R, p; 0, 1].
%
%   Maps body twists between frames: V_a = Ad(H_ab) * V_b.
%   See Murray, Li & Sastry, "A Mathematical Introduction to Robotic
%   Manipulation", Proposition 2.12.
%
%   Input:  T - 4x4 SE(3) matrix.
%   Output: AdT - 6x6 adjoint matrix.
    R = T(1:3,1:3);
    p = T(1:3,4);
    p_hat = vt.se3.hat3(p);
    AdT = [R, zeros(3); p_hat*R, R];
end
