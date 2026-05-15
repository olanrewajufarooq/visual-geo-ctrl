function M = coadP(P)
%COADP Symmetric coadjoint-like map for a momentum covector.
%   coadP(P) = [tilde(P_omega), tilde(P_v); tilde(P_v), 0_{3x3}]
%   for P = [P_omega; P_v] in R^6.
    P = P(:);
    assert(numel(P) == 6, 'fth:coadP: P must have 6 elements');
    M = [fth.se3.vec2tilde(P(1:3)), fth.se3.vec2tilde(P(4:6)); ...
         fth.se3.vec2tilde(P(4:6)), zeros(3)];
end
