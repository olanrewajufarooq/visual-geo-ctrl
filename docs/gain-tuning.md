# Adaptive Gain Tuning

`run/optimize_gains.m` tunes replay-trajectory gains with MATLAB's
Global Optimization Toolbox `particleswarm`. Parallel particle evaluation uses
Parallel Computing Toolbox when `opts.useParallel` is true.

The script runs twelve independent optimizations: six controller/adaptation
combinations, each with a no-payload and a payload-drop case:

- Nominal/basic
- Nominal/consistent
- Adaptive/basic/Euclidean
- Adaptive/basic/Bregman
- Adaptive/consistent/Euclidean
- Adaptive/consistent/Bregman

The optimization runs are separate because the cases use different controller
and adaptation settings. Gamma also has different shapes:

- Bregman: scalar `Gamma`, 19 variables total.
- Euclidean: 10-element `Gamma`, 28 variables total. The vector corresponds to
  `[m,hx,hy,hz,Ixx,Iyy,Izz,Ixy,Ixz,Iyz]`.

Each result optimizes one gain vector for one trajectory case. No-payload cases
use `vehicle-slight-dev`; payload-drop cases use `mid-vehicle-payload`. The
objective is a normalized weighted sum of
position RMSE, orientation RMSE, maximum position error, and normalized wrench
effort.

Run from the repository root:

```matlab
optimize_gains
```

Set `opts.scenarios` in `run/optimize_gains.m` to `'all'`, one scenario ID, or a
cell array of scenario IDs. Results are written to
`results/tuning/<timestamp>/<scenario-id>/`, including an executable
`best_gains.m` containing `Kp`, `Kd`, `lambda`, and `Gamma`.

The optimizer checkpoints each iteration in
`results/tuning/cache/<scenario-id>/optimizer_state.mat`. Cache selection is
per scenario. Set the `clearCache` field for the relevant entry in the
`GainOptimizationScenario.catalog()` method to `true` to force fresh
optimization; an empty field inherits `opts.clearCache`. Otherwise a matching
incomplete scenario resumes from its last saved swarm, and a completed one
prints its saved gains. Timestamped report folders remain separate, and are
preserved.
