# Run scripts

Every `.m` file here is a MATLAB script. Edit its settings at the top, then run it from the repository root after `startup`.

- `run_theory_suite.m` runs nominal, Euclidean, and Bregman variants for both `C1` and `C2`; each result is saved under `results/v2/`.
- `run_batch.m` sends the same independent scenario matrix through the sequential or Parallel Computing Toolbox batch runner.
- `optimize_gains.m` uses Global Optimization Toolbox to tune the transverse gains of the Bregman/`C2` scenario against explicit replay tracking and effort costs.
- `replay_run.m` visualizes a saved v2 result and can export an MP4.
