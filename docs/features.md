# Features

This framework is organized around a simulation pipeline that starts from `fth.sim.Config`, builds the scenario in `fth.sim.SimRunner`, and evaluates the resulting closed-loop dynamics on SE(3).

## Simulation Flow

```text
Config -> SimRunner.setup() -> SimRunner.run()
```

## Dynamics

- Full 6-DOF rigid-body dynamics implemented on SE(3)
- Euler-Poincare style body-frame modeling
- Optional ground contact with stiffness, damping, and friction terms
- Tracking metrics and logging integrated into the simulation loop

Core files:

- `src/+fth/+core/Dynamics.m`
- `src/+fth/+core/Logger.m`
- `src/+fth/+core/TrackingMetrics.m`

## Trajectory Generation

Built-in trajectory presets include:

- `hover`
- `circle`
- `infinity`
- `lissajous3d`
- `helix3d`
- `poly3d`
- `takeoffland`

Trajectory generation lives in `src/+fth/+traj/` and is centered on `AnalyticTraj`, `TrajectoryBase`, `TrajectoryFactory`, and `TimeScaling`.

## Controllers

The framework supports multiple wrench-generation strategies:

- `PD`
- `Feedforward`
- `FeedLin`

These are implemented through `src/+fth/+ctrl/ControllerWrench.m` with supporting factories and regressor helpers under `src/+fth/+ctrl/`.

## Adaptive Estimation

Supported adaptation modes:

- `none`
- `euclidean`
- `bregman`

The adaptive stack estimates inertial parameters online and includes a no-adaptation fallback, Euclidean updates, and Bregman divergence based updates.

Key files:

- `src/+fth/+ctrl/+adapt/NoAdaptation.m`
- `src/+fth/+ctrl/+adapt/EuclideanAdaptation.m`
- `src/+fth/+ctrl/+adapt/BregmanDivAdaptation.m`

## Potentials and Coriolis Options

Potential functions include:

- `log`
- `inertia-gain`
- `body-gain`
- `ref-gain`
- `sym-inv`

Coriolis factorization options include:

- `basic`
- `consistent`

These behaviors are split into dedicated factory-backed folders under `src/+fth/+ctrl/+potential/` and `src/+fth/+ctrl/+coriolis/`.

## Visualization and Analysis

- Live plotting and summary plots through `fth.plot.Plotter`
- 3D vehicle view through `fth.plot.UrdfViewer`
- Reference-only plotting through `run/plot_trajectories.m`
- Saved figures and logs under `results/`

URDF-backed rendering uses assets from `assets/hexacopter_description/urdf/`. When that path or the required toolbox is unavailable, the plotting stack can fall back to lighter visualization paths.
