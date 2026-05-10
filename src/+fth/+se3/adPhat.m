function M = adPhat(P)
%ADPHAT Symmetric adjoint-like map for a momentum covector.
%   adPhat(P) = [hat(P_omega), hat(P_v); hat(P_v), 0_{3x3}]
%   for P = [P_omega; P_v] in R^6.
%
%   Input:  P - 6x1 momentum (or any covector) [angular; linear].
%   Output: M - 6x6 matrix.
    M = [fth.se3.hat3(P(1:3)), fth.se3.hat3(P(4:6)); ...
         fth.se3.hat3(P(4:6)), zeros(3)];
end
