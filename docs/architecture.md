# Architecture

`agc.sim.run_scenario` is the single closed-loop simulation execution path. A scenario carries true vehicle/payload inertial parameters, initial state and estimate, reference trajectory replay sampler, controller gains, and three fixed rates:
- **500 Hz** physics integration (`dtPlant = 0.002` s)
- **100 Hz** adaptive parameter estimation (`dtAdaptation = 0.01` s)
- **50 Hz** geometric wrench control computation (`dtControl = 0.02` s)

The runner holds computed body wrenches with a zero-order hold (ZOH), applies them to the floating-base vehicle link in `PyBulletPlant`, and logs paper-facing diagnostics.

## Gravity Conventions
Gravity is explicit at the interface:
- `controller["gravity"] = np.array([0.0, 0.0, 9.81])` follows the paper's upward compensation-wrench convention.
- `plantGravity = np.array([0.0, 0.0, -9.81])` is the downward physical world acceleration supplied to PyBullet.
They must not be conflated.

## Inertial Estimates & Perturbations
Default theory scenarios initialize nominal control at the true loaded inertial parameters. Euclidean and Bregman adaptation instead share a deterministic 2% affine-invariant pseudo-inertia perturbation:
- Euclidean adaptation tracks the 10-parameter vector $\hat{\pi} \in \mathbb{R}^{10}$.
- Bregman adaptation tracks the SPD $4 \times 4$ pseudo-inertia matrix $\hat{\mathcal{J}} \in \mathcal{S}_{++}^4$ using the affine-invariant matrix exponential update, guaranteeing positive definiteness for all finite time.

## Mathematical Core (`agc.paper`)
`agc.paper` is strictly function-based and pure Python (requiring only NumPy and SciPy, not PyBullet). It exposes:
- Metric-compatible Coriolis factorizations: `c1` (Levi-Civita connection) and `c2` (coadjoint factorization).
- Tracking error covectors on $\mathrm{SE}(3)$.
- 6x10 rigid-body regressor matrix $Y(R, p, V, \dot{V}_r)$.
- Discrete adaptation laws for Euclidean gradient descent and Riemannian Bregman flows.

## Physical Plant (`agc.plant`)
`PyBulletPlant` implements the forward dynamics using PyBullet's multi-body contact solver and floating-base physics. It features:
- Procedural multi-link vehicle URDF with exact physical mass, center of mass, and inertia tensor.
- Dynamic payload detachment at the exact configured `releaseTime` boundary.
- Headless simulation (`p.DIRECT`) for fast batch sweeps and optimization, or GUI mode (`p.GUI`) with camera tracking and flight trails.

## Visualization and Persistence
- `agc.io`: Saves simulation runs to `run.npz` with comprehensive `metadata.json` and suite-level `manifest.json`.
- `agc.viz`: Post-processing visualization generating publication-quality figures for tracking RMSE, parameter convergence, C1 vs C2 comparisons, and Euclidean vs Bregman comparisons.
