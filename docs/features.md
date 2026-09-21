# Features

`adaptive-geo-ctrl-pybullet` provides an end-to-end framework for adaptive geometric tracking control of fully actuated UAVs on $\mathrm{SE}(3)$.

## Key Capabilities

1. **Lie Group & Geometric Mechanics (`agc.math`)**
   - Full $\mathrm{SE}(3)$ and $\mathrm{SO}(3)$ matrix Lie algebra operations, skew/unskew operators, group inverse, and spatial adjoint transforms.
   - $4 \times 4$ symmetric positive definite (SPD) pseudo-inertia $\mathcal{J}$ representation and conversions with the 10-parameter vector $\pi \in \mathbb{R}^{10}$.

2. **Metric-Compatible Adaptive Control (`agc.paper`)**
   - Paper-aligned tracking controller with transverse damping on $\mathrm{SE}(3)$.
   - Levi-Civita connection Coriolis factorization (`c1`) and skew-symmetric coadjoint factorization (`c2`).
   - Euclidean gradient parameter estimation.
   - Riemannian Bregman divergence estimation using geodesic flow on the SPD cone $\mathcal{S}_{++}^4$, mathematically guaranteeing positive definiteness for all finite time.

3. **Physics Simulation Engine (`agc.plant`)**
   - Floating-base multi-body forward dynamics in PyBullet.
   - Exact physical mass, inertia, and center of mass.
   - In-flight payload attachment and dynamic detachment at a configured timestamp.
   - Headless simulation (`p.DIRECT`) and 3D visual rendering (`p.GUI`) with real-time trajectory visualization and tracking camera.

4. **Multi-Rate Architecture (`agc.sim`)**
   - Physics integration at 500 Hz.
   - Parameter estimation at 100 Hz.
   - Control wrench updates at 50 Hz.
   - Zero-order hold (ZOH) actuator modeling.

5. **Staged Gain Optimization (`agc.opt`)**
   - Derivative-free block-coordinate optimization.
   - Multi-core Particle Swarm Optimization (PSO) and Differential Evolution (DE).
   - Hierarchical 7-stage and classic 4-stage optimization schedules.
   - Automated gain promotion to `config/optimized_gains.py`.

6. **Batch Comparative Evaluation & Persistence (`agc.batch`, `agc.io`, `agc.viz`)**
   - Automated 6-variant comparative study suite execution.
   - Dense time-series persistence in `.npz` with comprehensive `metadata.json`.
   - Publication-quality Matplotlib figures.
