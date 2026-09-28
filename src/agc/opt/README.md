# Staged Gain Optimization & Algorithms (`agc.opt`)

This package implements derivative-free, staged block-coordinate gain optimization for geometric SE(3) tracking, sliding surface manifold kinematics, and online parameter estimators.

---

## 1. Package Architecture

| Module | Purpose |
| :--- | :--- |
| [`pso.py`](file:///C:/Users/g202404900/Desktop/adaptive-geo-ctrl-pybullet/src/agc/opt/pso.py) | Vectorized, multi-process parallel Particle Swarm Optimizer with Clerc-Kennedy constriction and seed injection. |
| [`staged_optimizer.py`](file:///C:/Users/g202404900/Desktop/adaptive-geo-ctrl-pybullet/src/agc/opt/staged_optimizer.py) | Two-phase staged block-coordinate optimizer orchestrating hierarchical and classic optimization schedules. |
| [`bounds.py`](file:///C:/Users/g202404900/Desktop/adaptive-geo-ctrl-pybullet/src/agc/opt/bounds.py) | Search bounds in $\log_{10}$ and linear physical coordinates, stage schedules, and block coordinate indices. |
| [`objective.py`](file:///C:/Users/g202404900/Desktop/adaptive-geo-ctrl-pybullet/src/agc/opt/objective.py) | Multi-condition generalization cost function evaluating normalized tracking errors, parameter estimation, and actuation effort. |
| [`bregman_profile.py`](file:///C:/Users/g202404900/Desktop/adaptive-geo-ctrl-pybullet/src/agc/opt/bregman_profile.py) | High-resolution 1D log-grid diagnostic profiling for the scalar Bregman adaptation rate $\gamma_B$. |
| [`encoding.py`](file:///C:/Users/g202404900/Desktop/adaptive-geo-ctrl-pybullet/src/agc/opt/encoding.py) | Bidirectional mapping between optimizer decision vectors $\mathbf{x} \in \mathbb{R}^D$ and controller config dictionaries. |
| [`paper_adaptation.py`](file:///C:/Users/g202404900/Desktop/adaptive-geo-ctrl-pybullet/src/agc/opt/paper_adaptation.py) | Estimator dispatch and trajectory evaluation during candidate scoring. |

---

## 2. Particle Swarm Optimization (PSO)

### Mathematical Formulation
The optimization engine in [`pso.py`](file:///C:/Users/g202404900/Desktop/adaptive-geo-ctrl-pybullet/src/agc/opt/pso.py) implements the **Clerc-Kennedy Type 1" Constriction Particle Swarm Optimization** algorithm, matching the canonical behavior of MATLAB's Global Optimization Toolbox (`particleswarm`).

For a swarm of $P$ particles in dimension $D$, each particle $i \in \{1, \dots, P\}$ maintains position $\mathbf{x}_i^k \in \mathbb{R}^D$, velocity $\mathbf{v}_i^k \in \mathbb{R}^D$, and individual best location $\mathbf{p}_i^k \in \mathbb{R}^D$. Let $\mathbf{g}^k \in \mathbb{R}^D$ be the swarm's global best position at iteration $k$.

The constriction velocity update is:
$$v_{i,d}^{k+1} = \chi \left( v_{i,d}^k + c_1 r_{1,d}^k (p_{i,d}^k - x_{i,d}^k) + c_2 r_{2,d}^k (g_d^k - x_{i,d}^k) \right)$$

$$x_{i,d}^{k+1} = x_{i,d}^k + v_{i,d}^{k+1}$$

where:
- $r_{1,d}^k, r_{2,d}^k \sim \mathcal{U}(0, 1)$ are independent uniform random variables.
- $\phi = \phi_1 + \phi_2 > 4$ is the total acceleration coefficient (default: $\phi_1 = 2.05, \phi_2 = 2.05 \implies \phi = 4.1$).
- $\chi$ is the **Clerc constriction factor**:
  $$\chi = \frac{2}{|2 - \phi - \sqrt{\phi^2 - 4\phi}|} = \frac{2}{|2 - 4.1 - \sqrt{4.1^2 - 4(4.1)}|} \approx 0.72984$$
- The cognitive and social acceleration coefficients are:
  $$c_1 = \chi \phi_1 = 0.72984 \times 2.05 \approx 1.49618$$
  $$c_2 = \chi \phi_2 = 0.72984 \times 2.05 \approx 1.49618$$

### Convergence Properties
Unlike traditional inertia-weight formulations requiring heuristic velocity clamping ($v_{\max}$), the constriction model mathematically guarantees that the eigenvalues of the dynamic transition matrix lie strictly within the unit circle:
$$\max_j |\lambda_j| < 1$$
This ensures that particle trajectories oscillate with decaying amplitude toward the local attractor $\frac{c_1 \mathbf{p}_i + c_2 \mathbf{g}}{c_1 + c_2}$ without exploding into numerical instability.

### Implementation Enhancements
1. **Parallel Evaluation**: Batched objective evaluation across worker processes using Python's `concurrent.futures.ProcessPoolExecutor`.
2. **Seed Injection**: Deterministic injection of known high-quality candidates (registered best gains, manual baselines, and historical seeds) into particle positions at iteration 0.
3. **Stall-Based Termination**: Early stopping when the incumbent cost fails to improve by more than tolerance $\tau = 10^{-3}$ for $S_{\max} = 10$ consecutive iterations.

### Academic Literature / Citations for PSO
- **Kennedy, J., & Eberhart, R. (1995)**. *Particle swarm optimization*. In *Proceedings of ICNN'95 - International Conference on Neural Networks*, Perth, WA, Australia, Vol. 4, pp. 1942–1948.  
  DOI: [10.1109/ICNN.1995.488968](https://doi.org/10.1109/ICNN.1995.488968)  
  *(Foundational paper introducing the Particle Swarm Optimization paradigm).*
- **Clerc, M., & Kennedy, J. (2002)**. *The particle swarm - explosion, stability, and convergence in a multidimensional complex space*. *IEEE Transactions on Evolutionary Computation*, 6(1), pp. 58–73.  
  DOI: [10.1109/4235.985692](https://doi.org/10.1109/4235.985692)  
  *(Seminal theoretical analysis deriving the constriction coefficient $\chi = 0.72984$ and $c_1 = c_2 = 1.49618$ implemented in this library).*
- **Eberhart, R. C., & Shi, Y. (2000)**. *Comparing inertia weights and constriction factors in particle swarm optimization*. In *Proceedings of the 2000 Congress on Evolutionary Computation*, La Jolla, CA, USA, Vol. 1, pp. 84–88.  
  DOI: [10.1109/CEC.2000.870279](https://doi.org/10.1109/CEC.2000.870279)  
  *(Comparative empirical study establishing superior convergence and stability of constriction factors over inertia weights).*

---

## 3. Hierarchical Block-Coordinate Staged Optimization

### Theoretical Motivation
The geometric control and adaptation pipeline operates over 15 to 25 heterogeneous parameters:
$$\mathbf{x} = [\underbrace{K_R, K_\xi}_{\text{Outer-loop tracking}}, \underbrace{\Lambda_R, \Lambda_p}_{\text{Sliding metric}}, \underbrace{k_d, k_s, \alpha}_{\text{Dissipation}}, \underbrace{\gamma_E \text{ or } \gamma_B}_{\text{Online adaptation}}] \in \mathbb{R}^D$$

Optimizing all dimensions simultaneously in a single global swarm encounters:
1. **The Curse of Dimensionality**: High-dimensional search spaces require exponential particle counts to maintain coverage.
2. **Coupled Non-Convex Interactions**: Outer-loop tracking stiffness ($K_R, K_\xi$) defines the nominal tracking error kinematics, whereas sliding metric ($\Lambda$) defines the manifold $s = \dot{e} + \Lambda e$, and dissipation ($k_d, k_s, \alpha$) shapes the finite-time reaching phase. Perturbing all layers simultaneously introduces destructive interference.
3. **Estimator Bias**: If tracking gains and adaptation rates are optimized together, the optimizer can "cheat" by using aggressive proportional stiffness to compensate for an inert or unstable estimator, masking true adaptive estimation capability.

### Block Coordinate Descent (BCD) Formulation
The staged optimizer implements **Block Coordinate Descent (BCD)** (also known as *Alternating Optimization* or *Nonlinear Block Gauss-Seidel*).

Let the parameter space be partitioned into $M$ coordinate blocks $\mathcal{B}_1, \mathcal{B}_2, \dots, \mathcal{B}_M$ such that $\bigcup_{m=1}^M \mathcal{B}_m = \{1, \dots, D\}$. At stage $m$, only the coordinates in $\mathcal{B}_m$ are optimized while all complementary coordinates $\mathbf{x}_{\setminus \mathcal{B}_m}$ remain clamped to their incumbent values:
$$\mathbf{x}_{\mathcal{B}_m}^{(k+1)} = \arg\min_{\mathbf{z} \in \Omega_{\mathcal{B}_m}} J\left(\mathbf{x}_{\mathcal{B}_1}^{(k+1)}, \dots, \mathbf{x}_{\mathcal{B}_{m-1}}^{(k+1)}, \mathbf{z}, \mathbf{x}_{\mathcal{B}_{m+1}}^{(k)}, \dots, \mathbf{x}_{\mathcal{B}_M}^{(k)}\right)$$

### Staged Optimization Schedules
Defined in [`bounds.py`](file:///C:/Users/g202404900/Desktop/adaptive-geo-ctrl-pybullet/src/agc/opt/bounds.py):

#### 1. Nominal Mode (`--mode nominal`)
Optimizes the 15 baseline parameters across the full generalization matrix:
- **Hierarchical Schedule (6 stages)**:
  1. `all`: Global exploration across all 15 parameters to identify promising attractors.
  2. `tracking`: Attitudes ($K_R \in \mathbb{R}^3$) and positions ($K_\xi \in \mathbb{R}^3$), establishing SE(3) stiffness.
  3. `sliding_dissipation`: Joint sliding manifold kinematics ($\Lambda \in \mathbb{R}^6$) and dissipation ($k_d, k_s, \alpha$).
  4. `sliding_metric`: Metric tensor eigenvalues ($\Lambda_R, \Lambda_p$).
  5. `dissipation`: Reaching rate ($k_s$), linear damping ($k_d$), and fractional exponent ($\alpha$).
  6. `all`: Final joint polishing across all nominal parameters.
- **Classic Schedule (1 stage)**:
  1. `all`: Direct single-stage full-dimensional optimization.

#### 2. Adaptive Mode (`--mode adaptive`)
Adaptive optimization creates three independent gain records: `adaptive_base`, `euclidean`, and `bregman`. The `adaptive_base` tracking gains are optimized separately from nominal gains. That 15-coordinate base objective is the arithmetic mean across all selected conditions for both Euclidean and Bregman estimators. With defaults, that is one replay, two payload profiles, and both `lc` and `rb` connections for each estimator (eight scored runs total).

After the adaptive base is optimized, its tracking gains are frozen while the optimizer specializes the estimator parameters:
- **Euclidean ($\gamma_E \in \mathbb{R}^{10}$)**: 10 individual adaptation rates in $\log_{10}$ space $[-5, 0]$.
- **Bregman ($\gamma_B \in \mathbb{R}$)**: Evaluated via high-resolution 1D log-grid diagnostic profile in $[-5, -1]$ ($10^{-5}$ to $0.1$).

The Euclidean and Bregman entries share the optimized `adaptive_base` tracking gains, while `nominal` remains an independently optimized record. The estimator-specialization stages use every selected condition, including both `lc` and `rb`; the final `all` stage is omitted for adaptive controllers.

### Academic Literature / Citations for Hierarchical Optimization & BCD
- **Block Coordinate Descent Foundations**:
  - **Tseng, P. (2001)**. *Convergence of a block coordinate descent method for nondifferentiable minimization*. *Journal of Optimization Theory and Applications*, 109(3), pp. 475–494.  
    DOI: [10.1023/A:1017501703105](https://doi.org/10.1023/A:1017501703105)  
    *(Classic convergence framework establishing asymptotic convergence of block-coordinate updates under non-differentiable / non-convex constraints).*
  - **Wright, S. J. (2015)**. *Coordinate descent algorithms*. *Mathematical Programming*, 151(1), pp. 3–34.  
    DOI: [10.1007/s10107-015-0892-3](https://doi.org/10.1007/s10107-015-0892-3)  
    *(Comprehensive survey on modern deterministic and randomized block-coordinate descent methods).*
  - **Bezdek, J. C., & Hathaway, R. J. (2002)**. *Some notes on alternating optimization*. In *Advances in Soft Computing — AFSS 2002*, Lecture Notes in Computer Science, Vol. 2275, pp. 288–300. Springer, Berlin, Heidelberg.  
    DOI: [10.1007/3-540-45493-4_39](https://doi.org/10.1007/3-540-45493-4_39)  
    *(Foundational mathematical analysis of alternating optimization and local minima stability).*
  - **Razaviyayn, M., Hong, M., & Luo, Z.-Q. (2013)**. *A unified convergence analysis of block successive upper-bound minimization methods*. *SIAM Journal on Optimization*, 23(2), pp. 1126–1153.  
    DOI: [10.1137/120891009](https://doi.org/10.1137/120891009)  
    *(Unified convergence proofs for block successive optimization schemes).*
- **Hierarchical & Multistage Tuning in Robotics & Control**:
  - **Alvarez-Ramirez, J., Valencia, I., & Puebla, H. (2004)**. *Multistage and hierarchical optimization of PID controllers*. *Industrial & Engineering Chemistry Research*, 43(24), pp. 7842–7849.  
    DOI: [10.1021/ie049646n](https://doi.org/10.1021/ie049646n)  
    *(Demonstrates that decomposing controller parameter search into hierarchical sub-stages outperforms joint high-dimensional tuning).*
  - **Lee, T., Leok, M., & McClamroch, N. H. (2010)**. *Geometric tracking control of a quadrotor UAV on SE(3)*. In *49th IEEE Conference on Decision and Control (CDC)*, Atlanta, GA, USA, pp. 5420–5425.  
    DOI: [10.1109/CDC.2010.5717652](https://doi.org/10.1109/CDC.2010.5717652)  
    *(Provides the geometric SE(3) tracking control foundation justifying the separation of attitude stiffness $K_R$ from translational kinematics $K_\xi$).*
  - **Slotine, J.-J. E., & Li, W. (1987)**. *On the adaptive control of robot manipulators*. *The International Journal of Robotics Research*, 6(3), pp. 49–59.  
    DOI: [10.1177/027836498700600303](https://doi.org/10.1177/027836498700600303)  
    *(Seminal work establishing composite sliding-mode adaptive control and the decoupling of sliding surface kinematics from online parameter updates).*

---

## 4. Multi-Condition Generalization & Coriolis Invariance

### Training Matrix (4 Conditions per Estimator)
Candidates are evaluated against the arithmetic mean cost across a Cartesian generalization matrix. The default selects one replay:
$$\mathcal{T} = \{\text{lemniscate\_02\_auto}\} \times \{\text{flat\_light}, \text{tall\_heavy}\} \times \{\text{lc}, \text{rb}\}$$

For `adaptive_base`, the same four conditions are scored under each of the two estimators and all eight resulting records contribute equally to its objective. Replay selection can be expanded explicitly by the CLI.

A simulation crash, unbounded state, or constraint violation in *any* single condition marks the candidate as failed ($\text{cost} = \infty$). Replay `lemniscate_01_auto` with nominal payload is strictly held out for final paper evaluation.

### Coriolis Alignment & Adaptation Stability
1. **Tracking Invariance**: Base tracking gains ($K_R, K_\xi, \Lambda, k_d, k_s, \alpha$) are trained over both Levi-Civita (`lc`) and rigid-body (`rb`) Coriolis connections, guaranteeing Coriolis invariance.
2. **Connection Coverage**: Both estimator specializations, including Bregman, are scored under `lc` and `rb`. The nominal tracking invariance objective likewise covers both forms.

---

## 5. Usage & Execution

```powershell
# Activate conda environment
conda activate agc

# Full hierarchical optimization across all modes
python run/optimize_gains.py --mode all --schedule hierarchical --swarm-size 20 --max-iter 50

# Nominal tracking gain optimization only
python run/optimize_gains.py --mode nominal --schedule hierarchical --swarm-size 30 --max-iter 40

# Adaptive estimator tuning with frozen base tracking gains
python run/optimize_gains.py --mode adaptive --schedule hierarchical --swarm-size 20 --max-iter 40
```
