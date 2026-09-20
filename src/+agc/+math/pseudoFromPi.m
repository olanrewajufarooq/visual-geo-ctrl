function J = pseudoFromPi(pi)
%PSEUDOFROMPI Convert the ten inertial parameters to pseudo-inertia.
pi = pi(:);
I6 = agc.math.inertiaFromPi(pi);
Ib = I6(1:3,1:3);
S = 0.5 * trace(Ib) * eye(3) - Ib;
J = [S, pi(2:4); pi(2:4).', pi(1)];
J = 0.5 * (J + J.');
end
