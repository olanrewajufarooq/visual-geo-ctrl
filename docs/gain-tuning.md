# Unified Staged Gain Optimization

`run/optimize_gains.py` tunes controller and adaptation gains using staged derivative-free block-coordinate optimization powered by native parallelized Particle Swarm Optimization (PSO).

## Two-Mode Optimization Architecture

Optimization is decoupled into two principal modes to prevent estimator bias:

1. **`--mode nominal`**: Optimizes the 15 base tracking, sliding metric, and dissipation parameters across the 8-condition generalization matrix ($2\ \text{paths} \times 2\ \text{payloads} \times 2\ \text{Coriolis forms}$). Tracking gains are synchronized across all controller modes (`nominal`, `euclidean`, `bregman`) and Coriolis forms (`lc`, `rb`).
   - Hierarchical schedule (6 stages): `["all", "tracking", "sliding_dissipation", "sliding_metric", "dissipation", "all"]`
   - Classic schedule (1 stage): `["all"]`

2. **`--mode adaptive`**: First runs stages 1–5 on the nominal plant to establish optimal base tracking gains, freezes them, and then tunes the adaptation parameters in stage 6 (`adaptive`): $\gamma_E$ for Euclidean and $\gamma_B$ for Bregman.
   - Hierarchical schedule (6 stages): `["all", "tracking", "sliding_dissipation", "sliding_metric", "dissipation", "adaptive"]`
   - Classic schedule (3 stages): `["all", "nonadaptive", "adaptive"]`
   - **Estimator Fairness**: The final `"all"` stage is strictly omitted for adaptive controllers to prevent estimator bias and maintain pure, isolated tuning of adaptation dynamics.

## Gain Parameters & Blocks

The controller and adaptation gain vector includes:
- **$K_R \in \mathbb{R}^3$**: Attitude tracking gain diagonal (`KRdiag`)
- **$K_\xi \in \mathbb{R}^3$**: Position tracking gain diagonal (`Kxidiag`)
- **$\Lambda \in \mathbb{R}^6$**: Generalized metric damping matrix diagonal (`LambdaDiag`)
- **$k_s \in \mathbb{R}$**: Sliding surface scale (`ks`)
- **$k_d \in \mathbb{R}$**: Damping gain (`kd`)
- **$\alpha \in \mathbb{R}$**: Finite-time reaching exponent (`alpha`)
- **$\gamma_E \in \mathbb{R}^{10}$**: Euclidean adaptation gain vector (for Euclidean mode)
- **$\gamma_B \in \mathbb{R}$**: Bregman adaptation learning rate (for Bregman mode)

## Coriolis Invariance

Gains are invariant to the Coriolis factorization form (`lc` vs `rb`). During candidate evaluation, each candidate is scored over an 8-condition Cartesian product combining both Coriolis forms ($2 \times 2 \times 2$), and promoted gains are shared across both forms in the gain registry.

## Generalization Training Matrix (8 Conditions)

By default, each gain candidate is evaluated on the mean cost over eight training conditions:
- **2 Replays**: `lemniscate_02_auto` and `lemniscate_03_auto`
- **2 Payloads**: `flat_light` (0.60 kg, $0.16 \times 0.10 \times 0.06$ m) and `tall_heavy` (0.90 kg, $0.10 \times 0.10 \times 0.16$ m)
- **2 Coriolis Factorizations**: `lc` (Levi-Civita connection) and `rb` (rigid body coadjoint connection)

A failure in any condition rejects the candidate. Evaluation replay `lemniscate_01_auto` with the 0.75 kg payload is strictly held out.

## Objective Function

The objective is a dimensionless, normalized weighted sum dividing tracking errors, parameter estimation errors, and wrench effort by physical tolerances:

$$J = w_p \left(\frac{\text{RMSE}_p}{\sigma_p}\right)^2 + w_R \left(\frac{\text{RMSE}_R}{\sigma_R}\right)^2 + w_m \left(\frac{\text{RMSE}_m}{\sigma_m}\right)^2 + w_{\text{com}} \left(\frac{\text{RMSE}_{\text{com}}}{\sigma_{\text{com}}}\right)^2 + w_I \left(\frac{\text{RMSE}_I}{\sigma_I}\right)^2 + w_\tau \left(\frac{\text{RMSE}_\tau}{\sigma_\tau}\right)^2$$

Priority is given to mass and center-of-mass estimation, followed by attitude/position tracking and control effort.

## Running Optimization

```powershell
# Optimize all modes (nominal tracking baseline followed by isolated adaptation tuning)
python run/optimize_gains.py --mode all --swarm-size 20 --max-iter 50

# Optimize nominal tracking gains only
python run/optimize_gains.py --mode nominal --swarm-size 30 --max-iter 40

# Optimize adaptive gains with frozen tracking gains
python run/optimize_gains.py --mode adaptive --swarm-size 20 --max-iter 30

# Fast dry-run without registry promotion
python run/optimize_gains.py --mode all --swarm-size 4 --max-iter 2 --duration 2.0 --no-promote
```

## Gain Promotion

When optimization completes and strictly improves upon the registered incumbent:
- The gains are rounded to 4 significant figures.
- Both the package gain registry `src/agc/config/optimized_gains.py` and the root convenience alias `config/optimized_gains.py` are automatically updated and kept in sync.
- Tracking gains from nominal optimization are automatically propagated to all controller modes and Coriolis realizations.
- Pass `--no-promote` to evaluate candidates without modifying the gain registries.
