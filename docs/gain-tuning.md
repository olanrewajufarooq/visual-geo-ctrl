# Staged Gain Optimization

`run/optimize_gains.py` tunes controller and adaptation gains using staged derivative-free block-coordinate optimization.

Global search can be performed using **Particle Swarm Optimization (PSO)** or **Differential Evolution (DE)**, with optional local **Nelder-Mead simplex polishing** on the final stage.

## Gain Parameters & Blocks

The controller and adaptation gain vector includes:
- **$K_R \in \mathbb{R}^3$**: Attitude tracking gain diagonal (`KRdiag`)
- **$K_\xi \in \mathbb{R}^3$**: Position tracking gain diagonal (`Kxidiag`)
- **$\Lambda \in \mathbb{R}^6$**: Generalized metric damping matrix diagonal (`LambdaDiag`)
- **$k_s \in \mathbb{R}$**: Sliding surface scale (`ks`)
- **$k_d \in \mathbb{R}$**: Damping gain (`kd`)
- **$\gamma_E \in \mathbb{R}^{10}$**: Euclidean adaptation gain vector (for Euclidean mode)
- **$\gamma_B \in \mathbb{R}$**: Bregman adaptation learning rate (for Bregman mode)

## Optimization Schedules

### 1. Hierarchical 7-Stage Schedule (`--schedule hierarchical`)
Progressively decouples tracking, damping, and adaptation:
1. `KR, Kxi`: Attitude and position tracking gains.
2. `Lambda, ks, kd`: Metric and damping gains.
3. `Lambda`: Fine metric tuning.
4. `ks, kd`: Damping fine tuning.
5. `gammaE` or `gammaB`: Adaptation learning rate.
6. `all`: Joint polish over all active gains.
7. `polish`: Optional Nelder-Mead simplex refinement.

### 2. Classic 4-Stage Schedule (`--schedule classic`)
Matches the paper's original 4-stage progression:
1. `all`: Global exploration over all parameters.
2. `nonadaptive`: Fixed-gain block ($K_R, K_\xi, \Lambda, k_s, k_d$).
3. `adaptive`: Adaptation gain block ($\gamma_E$ or $\gamma_B$).
4. `all`: Final joint parameter optimization.

## Objective Function

The objective is a dimensionless, normalized weighted sum dividing tracking errors, parameter estimation errors, and wrench effort by physical tolerances:

$$J = w_p \left(\frac{\text{RMSE}_p}{\sigma_p}\right)^2 + w_R \left(\frac{\text{RMSE}_R}{\sigma_R}\right)^2 + w_m \left(\frac{\text{RMSE}_m}{\sigma_m}\right)^2 + w_{\text{com}} \left(\frac{\text{RMSE}_{\text{com}}}{\sigma_{\text{com}}}\right)^2 + w_I \left(\frac{\text{RMSE}_I}{\sigma_I}\right)^2 + w_\tau \left(\frac{\text{RMSE}_\tau}{\sigma_\tau}\right)^2$$

Priority is given to mass and center-of-mass estimation, followed by attitude/position tracking and control effort.

## Running Optimization

```powershell
# Run hierarchical PSO optimization for Bregman C1
python run/optimize_gains.py --mode bregman --coriolis c1 --schedule hierarchical --swarm-size 50 --max-iter 20

# Run Differential Evolution with Nelder-Mead polish for Euclidean C2
python run/optimize_gains.py --mode euclidean --coriolis c2 --method de --polish --seed 42
```

## Gain Promotion

When optimization completes and strictly improves upon the registered incumbent:
- The gains are rounded to 4 significant figures.
- The entry in `config/optimized_gains.py` is automatically updated.
- Pass `--no-promote` to evaluate candidates without modifying `config/optimized_gains.py`.
