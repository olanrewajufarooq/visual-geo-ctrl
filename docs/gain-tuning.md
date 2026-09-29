# Unified Staged Gain Optimization

`run/optimize_gains.py` tunes controller and adaptation gains using staged derivative-free block-coordinate optimization powered by native parallelized Particle Swarm Optimization (PSO).

## Two-Mode Optimization Architecture

Optimization is decoupled into nominal tracking and adaptive tuning:

1. **`--mode nominal`**: Optimizes the 15 tracking, sliding metric, and dissipation parameters for the bare vehicle with payload-drop dynamics disabled. Defaults cover one replay and both Coriolis forms (`lc`, `rb`), for two conditions. Nominal gains are stored separately from adaptive tracking gains.
   - Hierarchical schedule (6 stages): `["all", "tracking", "sliding_dissipation", "sliding_metric", "dissipation", "all"]`
   - Classic schedule (1 stage): `["all"]`

2. **`--mode adaptive`**: First optimizes the shared `adaptive_base` tracking gains over the payload-release cases for both estimators, then freezes them and tunes Euclidean $\gamma_E$ and Bregman $\gamma_B$ separately. The default adaptive matrix includes one replay, two payload profiles, and both Coriolis forms (four conditions per estimator).
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

Gains are shared across Coriolis forms (`lc` and `rb`). Each candidate's objective is the arithmetic mean of its condition costs. A candidate is rejected if any selected condition fails. The exact training conditions are recorded in the run manifest and per-condition evaluation records.

## Training Conditions

The default uses one replay (`lemniscate_02_auto`) and both Coriolis factorizations (`lc` and `rb`). Nominal gains are trained on the bare vehicle with payload-drop dynamics disabled, so they have two conditions. Adaptive gains are trained across two payload profiles—`flat_light` (0.60 kg, $0.16 \times 0.10 \times 0.06$ m) and `tall_heavy` (0.90 kg, $0.10 \times 0.10 \times 0.16$ m)—with the payload released, giving four conditions per estimator. The shared adaptive base is scored with both Euclidean and Bregman estimators, averaging all eight records equally. A failure in any scored condition rejects the candidate. Evaluation replay `lemniscate_01_auto` with the 0.75 kg payload is held out.

## Objective Function

The objective is a dimensionless, normalized weighted sum dividing tracking errors, parameter estimation errors, force RMS, and torque RMS by their physical scales. Force and torque use separate scales and weights:

$$J = \cdots + w_F (F_{\mathrm{RMS}}/\sigma_F)^2 + w_\tau (\tau_{\mathrm{RMS}}/\sigma_\tau)^2$$

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

# Resume a compatible interrupted run from its unique output directory
python run/optimize_gains.py --mode all --swarm-size 20 --max-iter 50 --resume results/optimization/timestamped/<run-id>
```

## Gain Promotion

When optimization completes and strictly improves upon the registered incumbent:
- The gains are rounded to 4 significant figures. Each timestamped run writes a manifest, stage checkpoints, a run-scoped best candidate, and per-condition evaluation records. Timestamped output is the default; `--inplace-save` explicitly selects the shared `results/optimization/best-gain` directory.
- Promotion compares candidates after evaluating them under the current objective. A prior `optimizationCost` is never treated as directly comparable. When nominal tracking gains change, stored adaptive objective costs are cleared because they correspond to a different shared tracking controller.
- Gain promotion atomically updates the package registry `src/agc/config/optimized_gains.py`.
- Tracking gains from nominal optimization are automatically propagated to all controller modes and Coriolis realizations.
- Pass `--no-promote` to evaluate candidates without modifying the gain registries.
