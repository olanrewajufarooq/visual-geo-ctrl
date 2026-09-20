function Hinv = invSE3(H)
%INVSE3 Inverse homogeneous transform.
validateattributes(H, {'numeric'}, {'real', 'finite', 'size', [4 4]});
R = H(1:3,1:3); p = H(1:3,4);
Hinv = [R.', -R.' * p; 0, 0, 0, 1];
end
