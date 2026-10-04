# Gain Optimization

`run/optimize_gains.py` performs staged block-coordinate optimization for nominal, Euclidean-adaptive, and Bregman-adaptive controllers. Each mode and Coriolis form is an independent optimization target; the registry therefore contains six gain sets (`nominal_lc`, `nominal_rb`, `euclidean_lc`, `euclidean_rb`, `bregman_lc`, `bregman_rb`). No tracking gains are shared or propagated across modes or factorization forms.

## Search modes and schedules

`--mode all --coriolis all` visits the six mode/factorization pairs independently. The hierarchical schedule uses six stages for nominal and seven stages for adaptive modes, ending with an all-gain refinement. The classic schedule uses one `all` stage for nominal and four stages (`all`, `nonadaptive`, `adaptive`, `all`) for Euclidean and Bregman. During a Bregman adaptive stage, the scalar `gammaB` is selected through the logarithmic profile search. Global search defaults to PSO; DE and optional Nelder–Mead polishing are available from the CLI.

The CLI defaults are hierarchical schedule, PSO, swarm size 20, 50 iterations per stage, 10 stall iterations, 30 s simulations, parallel evaluations, and in-place checkpoint saving. `run/optimize_gains.py --help` lists all overrides.

## Training conditions and aggregation

By default, every selected mode/factorization pair is evaluated on the Cartesian product of three replay IDs (`lemniscate_02_auto`, `lemniscate_03_auto`, `lemniscate_04_auto`) and two payload profiles (`flat_light`, `tall_heavy`). Each condition includes payload release. The candidate objective is the arithmetic mean of the six condition costs; any failed condition rejects the candidate. Replay and profile overrides are recorded with the run artifacts.

## Objective

The candidate cost is the weighted sum of normalized position, attitude, mass, center-of-mass, linear-velocity, angular-velocity, inertia-estimation, and wrench-effort errors. Wrench effort uses the combined `wrenchRMS` metric with scale 50 and weight 0.5, matching the `ad38b3d` optimizer; force and torque are not separate objective terms. The objective weights and scales are defined in `src/agc/opt/objective.py`.

## Example commands

```powershell
# Optimize all six independent mode/factorization pairs
python run/optimize_gains.py --mode all --coriolis all --schedule hierarchical

# Run only Euclidean adaptation with the RB realization
python run/optimize_gains.py --mode euclidean --coriolis rb

# Use DE and Nelder–Mead polishing for Bregman LC
python run/optimize_gains.py --mode bregman --coriolis lc --method de --polish

# Override replay and payload-profile selection
python run/optimize_gains.py --train-replay-ids lemniscate_02_auto,lemniscate_03_auto --training-payload-profiles flat_light
```

Optimization artifacts and promotion are scoped to the selected mode/factorization. The paper runner only loads configured gains; it does not run optimization. For its payload-release comparison it uses the nominal-LC, Euclidean-LC, and Bregman-LC entries for the known-inertia, Euclidean, and Bregman controllers, respectively.
