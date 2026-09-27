# Gain-Tuning Data Output

Every optimization writes to a unique run directory under
`results/optimization/timestamped/<run-id>/` by default. The run ID includes
subsecond time and a random suffix, preventing concurrent or same-second runs
from overwriting each other. `--inplace-save` explicitly selects the shared
`results/optimization/best-gain/` location. An interrupted timestamped run can
be resumed with `--resume <run-directory>` when its objective, training
conditions, schedule, and PSO settings match the manifest.

## Output Files

- `manifest.json`: objective version and fingerprint, scoring weights and
  scales, code revision, selected replay/payload/Coriolis conditions,
  trajectory artifact hashes, relevant source-file hashes, runtime package
  versions, frozen initial registry seed candidates, duration, schedule,
  seed, and PSO settings. Its status records whether the run completed.
- `<mode>/optimization.json`: current incumbent, condition-mean cost, and
  completed stage history.
- `<mode>/best_gain.json`: the best candidate scored by this run, including
  per-condition metrics and failures, whether or not it improves the
  canonical registry candidate.
- `<mode>/stage_<index>_<name>.json`: atomically replaced checkpoint. PSO
  stages retain particle positions and velocities, personal/global bests,
  stall count, iteration history, and random-generator state. Completed
  stages are reused on resume.
- `results/optimization/best-gain/<mode>.json`: canonical incumbent used as
  the promotion benchmark. Its candidate is re-evaluated under the current
  objective before comparison; an older stored cost is not compared directly.

Promotion updates `src/agc/config/optimized_gains.py` atomically. If nominal
tracking gains change, adaptive registry costs are cleared because those
scores were obtained with different shared tracking gains.

## Gain Data Dictionary

| Gain Field | Dimension | Physical Meaning |
| --- | --- | --- |
| `KRdiag` | $3 \times 1$ | Diagonal attitude tracking gains on $\mathfrak{so}(3)$ |
| `Kxidiag` | $3 \times 1$ | Diagonal position tracking gains in $\mathbb{R}^3$ |
| `LambdaDiag` | $6 \times 1$ | Diagonal metric damping gains on $\mathfrak{se}(3)$ |
| `kd` | Scalar | Velocity damping gain |
| `ks` | Scalar | Generalized sliding surface scaling |
| `alpha` | Scalar | Fractional reaching exponent |
| `gammaE` | $10 \times 1$ | Euclidean adaptation learning rates |
| `gammaB` | Scalar | Bregman learning rate for the SPD pseudo-inertia update |
