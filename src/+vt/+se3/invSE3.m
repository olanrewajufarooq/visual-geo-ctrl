function Hinv = invSE3(H)
%INVSE3 Inverse of an SE(3) homogeneous transform.
%   H^{-1} = [R', -R'*p; 0, 1].  Avoids general 4x4 matrix inverse.
    R = H(1:3,1:3);
    p = H(1:3,4);
    Hinv = [R', -R'*p; 0 0 0 1];
end
