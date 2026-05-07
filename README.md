# adaptive-geo-ctrl

MATLAB simulation framework for geometric adaptive control of a fixed-tilt hexacopter on SE(3).

## Overview

This project implements and validates adaptive geometric controllers for a fully-actuated
fixed-tilt hexacopter. All dynamics use the Euler-Poincaré formulation on SE(3) (body-frame
rigid-body dynamics), and adaptation uses a regressor-based law that estimates the 10-parameter
inertial vector online.

The framework is designed for controller research: rapid iteration in MATLAB, reproducible
batch sweeps, and paper-quality result generation.

## Quick Start

```matlab
% Setup (run once per MATLAB session)
startup

% Nominal control demo
run_nominal_demo

% Adaptive control demo (with payload drop)
run_adaptive_demo

% Run unit tests
runtests('tests')

% Reproduce paper results
run_nominal_paper_ICUAS
run_adaptive_paper_ICUAS
```

## Architecture

**Simulation pipeline:** `Config` → `SimRunner.setup()` → `SimRunner.run()` → plot/save

**Key packages** (under `src/+fth/`):

| Package | Responsibility |
|---------|---------------|
| `+sim` | `Config` fluent builder, `SimRunner`, `BatchRunner` |
| `+core` | `Dynamics` (SE(3) rigid-body plant), `Logger`, `TrackingMetrics` |
| `+ctrl` | `WrenchController` (PD / FeedLin / Feedforward modes) |
| `+ctrl/+adapt` | Adaptation strategies: `NoAdaptation`, `EuclideanAdaptation`, `GeoAwareAdaptation` |
| `+ctrl/+potential` | Pose error potentials: `LieAlgebraPotential`, `SeparatePotential` |
| `+traj` | `PreComputedTrajectory`, `ModelReferenceTrajectory` |
| `+se3` | Lie group utilities: exp, log, Ad, hat, vee, inv |
| `+io` | `ConsoleCapture`, `ConsoleFormatter`, `NamingUtils`, `ResultsManager` |
| `+plot` | `Plotter`, `UrdfViewer` |
| `+utils` | Rotation conversions, inertia helpers, payload utilities |

## Coding Conventions

- **Namespace:** `fth.<package>.<Class>` (e.g. `fth.sim.SimRunner`, `fth.se3.expSE3(...)`)
- **Tests:** MATLAB unittest framework in `tests/`. Class names start with `Test`.
- **Commit style:** Conventional Commits (`feat`, `fix`, `refactor`, `test`, `chore`) with scope.
