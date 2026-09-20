function next = propagate(plant, state, wrench, dt)
%PROPAGATE Fixed-step rigid-body state update driven by Toolbox dynamics.
% Robotics System Toolbox supplies Vdot; the update only integrates the
% floating-base kinematics while preserving R in SO(3).
validateattributes(dt, {'numeric'}, {'real', 'finite', 'positive', 'scalar'});
Vdot = agc.plant.acceleration(plant, state, wrench);
Vmid = state.V(:) + 0.5 * dt * Vdot;
R = state.H(1:3,1:3); p = state.H(1:3,4);
Rmid = R * expm(agc.math.skew(Vmid(1:3)) * (0.5 * dt));
Rnext = R * expm(agc.math.skew(Vmid(1:3)) * dt);
pnext = p + dt * Rmid * Vmid(4:6);
next = struct('H', [Rnext, pnext; 0, 0, 0, 1], ...
              'V', state.V(:) + dt * Vdot);
end
