# Project Structure

## Repository Layout

```text
adaptive-geo-ctrl/
|-- assets/
|   `-- hexacopter_description/urdf/variable_tilt_hexacopter.urdf
|-- docs/
|-- run/
|   |-- ci_release.m
|   |-- plot_trajectories.m
|   |-- run_adaptive_coriolis_comparison.m
|   |-- run_adaptive_demo.m
|   |-- run_adaptive_gain_comparison.m
|   |-- run_nominal_coriolis_comparison.m
|   `-- run_nominal_demo.m
|-- src/+fth/
|   |-- +core/
|   |-- +ctrl/
|   |-- +io/
|   |-- +plot/
|   |-- +se3/
|   |-- +sim/
|   `-- +traj/
|-- tests/
|-- trajectories/
|-- .github/workflows/release-results.yml
|-- startup.m
`-- README.md
```

## Package Roles

| Package | Purpose |
| --- | --- |
| `fth.core` | Plant dynamics, logging, and tracking metrics |
| `fth.ctrl` | Controllers, regressors, Coriolis strategies, potentials, adaptation |
| `fth.io` | Result naming, console capture, and saved-run plotting |
| `fth.plot` | Live plots, summary figures, cumulative plots, URDF view |
| `fth.se3` | Lie-group and rigid-body math utilities |
| `fth.sim` | Config objects, simulation orchestration, batch expansion |
| `fth.traj` | Analytic trajectories, factories, time scaling |

## Key Classes and Files

| File | Role |
| --- | --- |
| `src/+fth/+sim/Config.m` | Main configuration object and batch-expansion entry point |
| `src/+fth/+sim/SimRunner.m` | Main simulation orchestrator |
| `src/+fth/+sim/BatchRunner.m` | Batch run execution and reporting helpers |
| `src/+fth/+core/Dynamics.m` | SE(3) rigid-body plant model |
| `src/+fth/+ctrl/ControllerWrench.m` | Wrench-level control computation |
| `src/+fth/+ctrl/RigidBodyRegressor.m` | Regressor for adaptive parameter estimation |
| `src/+fth/+traj/AnalyticTraj.m` | Built-in trajectory generation |
| `src/+fth/+plot/Plotter.m` | Summary and live plotting |
| `src/+fth/+plot/UrdfViewer.m` | URDF-backed vehicle visualization |
| `src/+fth/+io/ResultsManager.m` | Saved-result handling and replotting |

## Run Script Roles

| Script | Role |
| --- | --- |
| `run_nominal_demo.m` | Baseline scenario |
| `run_adaptive_demo.m` | Adaptive payload-drop scenario |
| `run_nominal_coriolis_comparison.m` | Nominal Coriolis comparison |
| `run_adaptive_coriolis_comparison.m` | Adaptive Coriolis comparison |
| `run_adaptive_gain_comparison.m` | Multi-trajectory adaptive gain sweep |
| `plot_trajectories.m` | Preview trajectory shapes |
| `ci_release.m` | Headless CI/release batch run |

## Tests

The `tests/` folder covers configuration, dynamics, SE(3) utilities, adaptation math, results handling, URDF viewing, and controller behavior. Use it as the first stop when changing public behavior in the framework.
