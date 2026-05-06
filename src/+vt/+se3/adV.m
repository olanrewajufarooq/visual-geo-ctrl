function adV = adV(V)
%ADV Small adjoint (Lie bracket) for a body twist.
%   ad_V = [hat(omega), 0; hat(v), hat(omega)]  for V = [omega; v].
%
%   Used in the Euler-Poincare equation: ad_V^T * I * V gives the
%   Coriolis/centripetal wrench.
%
%   Input:  V - 6x1 body twist [angular; linear].
%   Output: adV - 6x6 small adjoint matrix.
    w_hat = vt.se3.hat3(V(1:3));
    v_hat = vt.se3.hat3(V(4:6));
    adV = [w_hat, zeros(3); v_hat, w_hat];
end
