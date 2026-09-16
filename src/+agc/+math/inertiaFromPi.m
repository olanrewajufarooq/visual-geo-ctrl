function I6 = inertiaFromPi(pi)
%INERTIAFROMPI Generalized inertia for pi=[m; h; Ixx Iyy Izz Ixy Ixz Iyz].
pi = pi(:);
validateattributes(pi, {'numeric'}, {'real', 'finite', 'numel', 10});
m = pi(1); h = pi(2:4); Ip = pi(5:10);
Ib = [Ip(1), Ip(4), Ip(5); Ip(4), Ip(2), Ip(6); Ip(5), Ip(6), Ip(3)];
I6 = [Ib, agc.math.skew(h); -agc.math.skew(h), m * eye(3)];
end
