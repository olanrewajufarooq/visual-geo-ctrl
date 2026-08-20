# Getting Started

## Prerequisites

- MATLAB R2020b or later is recommended.
- Robotics System Toolbox is optional and mainly affects URDF-based visualization.
- A standard desktop MATLAB environment is enough for nominal runs; longer batch sweeps benefit from more CPU time and memory.

## Installation

Clone the repository:

```bash
git clone https://github.com/kfupm-arm-lab/adaptive-geo-ctrl.git
cd adaptive-geo-ctrl
```

Initialize the MATLAB path from the repository root:

```matlab
cd('path/to/adaptive-geo-ctrl')
startup
```

`startup.m` adds the repository root, `src/`, and `run/` to the active MATLAB path.

## First Run

Run a nominal demo:

```matlab
startup
run_nominal_demo
```

Run an adaptive demo with a scheduled payload change:

```matlab
startup
run_adaptive_demo
```

## Batch and Release Runs

For a larger comparison run:

```matlab
startup
run_adaptive_gain_comparison
```

For the CI-oriented release batch:

```matlab
startup
ci_release
```

## Available Run Scripts

| Script | Purpose |
| --- | --- |
| `run_nominal_demo.m` | Single-run nominal control example |
| `run_adaptive_demo.m` | Adaptive control with payload drop |
| `run_nominal_coriolis_comparison.m` | Compare nominal Coriolis formulations |
| `run_adaptive_coriolis_comparison.m` | Compare adaptive Coriolis formulations |
| `run_adaptive_gain_comparison.m` | Batch Euclidean and Bregman gain sweep |
| `plot_trajectories.m` | Plot reference trajectories without a simulation run |
| `ci_release.m` | Batch release scenario used by GitHub Actions |

## Typical Workflow

1. Call `startup`.
2. Build a `fth.sim.Config` object or use one of the `run/` scripts.
3. Finalize the config with `cfg.done()`.
4. Create `fth.sim.SimRunner`, call `setup()`, then `run(...)`.
5. Inspect figures and the generated `results/` folder.
