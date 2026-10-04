# VGC

VGC is a nominal geometric-control simulator and 3D visualizer for fully actuated vehicles. It uses NumPy/SciPy for SE(3) mathematics and PyBullet for rigid-body simulation.

## Quick start

```bash
pip install -e .[dev]
python run/run_sim.py --gui --replay-id lemniscate_01_auto
```

Use `--coriolis lc` or `--coriolis rb` to select the nominal model realization. Runs are saved under `results/inplace/nominal_<form>/` unless an output directory is supplied.

## Project layout

- `src/vgc`: simulation, nominal controller, plant, math, persistence, and visualization code.
- `run`: command-line entry points for simulation, batch comparison, replay, and trajectory plotting.
- `trajectories`: source and processed reference trajectories.
- `tests`: regression tests for the simulator and visualization tools.

The project intentionally contains one known-inertia controller mode and a bare vehicle model.
