function pi = piFromPseudo(J)
%PIFROMPSEUDO Convert a symmetric pseudo-inertia matrix to pi.
validateattributes(J, {'numeric'}, {'real', 'finite', 'size', [4 4]});
J = 0.5 * (J + J.');
S = J(1:3,1:3);
Ib = trace(S) * eye(3) - S;
pi = [J(4,4); J(1:3,4); Ib(1,1); Ib(2,2); Ib(3,3); ...
      Ib(1,2); Ib(1,3); Ib(2,3)];
end
