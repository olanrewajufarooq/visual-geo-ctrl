function A = adjointSE3(H)
%ADJOINTSE3 Adjoint matrix for twists ordered as [angular; linear].
validateattributes(H, {'numeric'}, {'real', 'finite', 'size', [4 4]});
R = H(1:3,1:3); p = H(1:3,4);
A = [R, zeros(3); agc.math.skew(p) * R, R];
end
