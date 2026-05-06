# AGENT.md

## Project Overview

MATLAB simulation framework for a variable-tilt hexacopter using SE(3) Lie group dynamics. Supports nominal and adaptive control research, batch simulation sweeps, and paper result reproduction for ICUAS publications.

## Quick Reference

```matlab
% Setup (run once per MATLAB session)
startup

% Run demos
run_nominal_demo
run_adaptive_demo

% Run unit tests
runtests('tests')

% Run paper reproduction
run_nominal_paper_ICUAS
run_adaptive_paper_ICUAS

% CI release scripts (used by GitHub Actions)
ci_nominal_release
ci_adaptive_release
```

## Architecture

**Simulation pipeline:** `Config` -> `SimRunner.setup()` -> `SimRunner.run()` -> plot/save

**Key packages** (under `src/+vt/`):
| Package | Responsibility |
|---------|---------------|
| `+config` | `Config` fluent builder for all simulation parameters |
| `+sim` | `SimRunner`, `BatchRunner`, `ResultsManager`, `ConsoleCapture`, `NamingUtils` |
| `+ctrl` | `WrenchController` (PD / FeedLin / Feedforward modes) |
| `+ctrl/+adapt` | Adaptation strategies: `NoAdaptation`, `EuclideanAdaptation`, `GeoAwareAdaptation` |
| `+ctrl/+potential` | Pose error potentials: `LieAlgebraPotential`, `SeparatePotential` |
| `+plant` | `HexacopterPlant` SE(3) rigid-body dynamics |
| `+traj` | `PreComputedTrajectory`, `ModelReferenceTrajectory` |
| `+se3` | Lie group utilities: exp, log, Ad, hat, vee, inv |
| `+metrics` | `TrackingMetrics` for RMSE/NRMSE/score computation |
| `+plot` | `Plotter` for summary and standalone figures |
| `+core` | `Logger` for time-series data capture |
| `+utils` | Rotation conversions, inertia helpers, payload utilities |

## Coding Conventions

- **MATLAB packages**: All production code lives under `src/+vt/+<package>/`. Reference via `vt.<package>.<Class>`.
- **Handle classes**: `SimRunner`, `BatchRunner`, `WrenchController`, `HexacopterPlant`, `Config` are handle classes (reference semantics). `Config.copy()` creates a detached value copy.
- **Fluent API**: `Config` setters return `obj` for chaining: `cfg.setTrajectory('circle').setController('PD').done()`.
- **Factory pattern**: `AdaptationFactory`, `PotentialFactory`, `TrajectoryFactory` create strategy objects from config.
- **SE(3) functions**: Standalone functions in `+se3/`, not a class. Call as `vt.se3.expSE3(...)`.
- **Tests**: MATLAB unittest framework in `tests/`. Class names start with `Test`.
- **No external toolboxes required** for core simulation. Robotics System Toolbox optional for URDF visualization.
- **Commit style**: Conventional Commits format (feat, fix, docs, style, refactor, test, chore) with scope: `feat(sim): add batch runner`.

## Key Design Decisions

- **Body-frame dynamics**: All dynamics use the Euler-Poincare equation on SE(3), not world-frame Newton-Euler.
- **6x6 generalized inertia** (`I6`): Encodes mass, CoG, and rotational inertia in a single matrix.
- **10-parameter adaptation**: theta = [I_xx, I_yy, I_zz, I_xy, I_yz, I_xz, m, m*CoG_x, m*CoG_y, m*CoG_z].
- **Multi-rate simulation**: `sim_dt` <= `adaptation_dt` <= `control_dt`. Auto-resolved by `Config.done()`.
- **Batch mode**: `Config` supports multi-trajectory (`setTrajectory({'circle','infinity'})`) and multi-gain sweeps (`setKpGains(Nx6)`). `BatchRunner` orchestrates child `SimRunner` instances.
- **Results persistence**: Each run saves `sim_data.mat` with `logs`, `metrics`, `est`, `runInfo`, `cfgSnapshot`. Organized under `results/nominal/` or `results/adaptive/`.

## Common Pitfalls

- Always call `cfg.done()` after configuration to resolve timesteps and validate gains.
- `Config` is a handle class: assigning `cfg2 = cfg` shares state. Use `cfg.copy()` for independence.
- Run `startup` before any script to ensure `src/` and `run/` are on the MATLAB path.
- Batch gain counts (Kp rows, Kd rows, Gamma rows) must match when more than one run is requested.
