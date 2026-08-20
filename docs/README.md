# Documentation Hub

This directory holds the detailed project documentation that used to live in the root README. Use the links below to jump to the part of the framework you need.

## Table of Contents

- [Getting Started](getting-started.md)
- [Configuration](configuration.md)
- [Batch Simulations](batch-simulations.md)
- [Features](features.md)
- [Project Structure](project-structure.md)
- [Simulation Outputs](simulation-outputs.md)
- [CI/CD](cicd.md)
- [Customization](customization.md)
- [Troubleshooting](troubleshooting.md)
- [Coding Conventions](coding-conventions.md)

## Recommended Reading Order

1. Read [Getting Started](getting-started.md) to install the project and run the first demo.
2. Read [Configuration](configuration.md) to understand `fth.sim.Config` and the main setup flow.
3. Read [Features](features.md) and [Project Structure](project-structure.md) for the simulation architecture.
4. Read [Batch Simulations](batch-simulations.md) and [Simulation Outputs](simulation-outputs.md) before large sweeps.
5. Read [Customization](customization.md) and [Coding Conventions](coding-conventions.md) before extending the codebase.

## Core Entry Points

- `startup` adds `src/` and `run/` to the MATLAB path.
- `run_nominal_demo` runs a baseline controller example.
- `run_adaptive_demo` runs an adaptive payload-drop example.
- `run_adaptive_gain_comparison` runs batch Euclidean and Bregman gain sweeps.
- `ci_release` runs the batch scenario used by the release workflow.

## Related Repository Files

- Root overview: [`README.md`](../README.md)
- License: [`LICENSE`](../LICENSE)
- Main startup script: [`startup.m`](../startup.m)
- MATLAB run scripts: [`run/`](../run/)
- Source package root: [`src/+fth/`](../src/+fth/)
