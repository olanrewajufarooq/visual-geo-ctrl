# Gain-Tuning Data Output

Staged optimization reports and checkpoints are written to `results/tuning/<mode>_<coriolis>_<timestamp>/`.

## Output Files

- `optimization_results.json`: Complete record of the optimization session, containing:
  - `mode`, `coriolis`, `schedule`, and `method` settings.
  - Initial baseline costs (`manual_cost`, `registered_cost`, `incumbent_seed_cost`).
  - Stage-by-stage progression with block names, candidate costs, improvement status, and elapsed wall-clock times.
  - Final optimized gain dictionary (`KRdiag`, `Kxidiag`, `LambdaDiag`, `kd`, `ks`, `alpha`, `gammaE`, `gammaB`).
  - Promotion status indicating whether registries (`src/agc/config/optimized_gains.py` and `config/optimized_gains.py`) were updated.
- `checkpoint_<stage>.json`: Intermediate checkpoint after each block optimization stage, preserving the best candidate state and cost history.

## Gain Data Dictionary

| Gain Field | Dimension | Physical Meaning |
| --- | --- | --- |
| `KRdiag` | $3 \times 1$ | Diagonal attitude tracking gains on $\mathfrak{so}(3)$ |
| `Kxidiag` | $3 \times 1$ | Diagonal position tracking gains in $\mathbb{R}^3$ |
| `LambdaDiag` | $6 \times 1$ | Diagonal metric damping gains on $\mathfrak{se}(3)$ |
| `kd` | Scalar | Velocity damping gain |
| `ks` | Scalar | Generalized sliding surface scaling |
| `alpha` | Scalar | Orientation sliding scale |
| `gammaE` | $10 \times 1$ | Euclidean adaptation learning rates: $[m, h_x, h_y, h_z, I_{xx}, I_{yy}, I_{zz}, I_{xy}, I_{xz}, I_{yz}]$ |
| `gammaB` | Scalar | Bregman Riemannian learning rate for SPD affine-invariant update |
