# Getting Started

## Prerequisites

- **Python 3.10+** (Python 3.12 recommended)
- **Miniconda** or **Anaconda**
- Recommended OS: Windows, Linux, or macOS

## Environment Installation

Create and activate the dedicated `agc` Conda environment:

```powershell
conda env create -f environment.yml
conda activate agc
```

Alternatively, install the package in editable mode with development dependencies:

```powershell
pip install -e ".[dev]"
```

Verify that the environment imports correctly:

```powershell
python -c "import agc.math, agc.paper, agc.plant; print('AGC imported successfully')"
```

---

## First Run

### 1. Interactive 3D Simulation
Run an adaptive tracking experiment in the 3D PyBullet GUI:

```powershell
python run/run_theory_suite.py --mode bregman --coriolis c1 --gui --speed 1.0
```

GUI mode opens the PyBullet scene together with a live Matplotlib diagnostics
window. The dashboard shows reference-versus-actual position, geometric
attitude error, linear and angular velocity, commanded wrench, sliding
variables, and estimated-versus-active mass, CoM, and inertia parameters.
Physics continues at the normal 500 Hz plant rate while visualization samples
at a lower rate, so stale dashboard frames may be dropped to preserve
simulation speed. The 3D scene keeps the reference path, vehicle trail, body
axes, altitude cue, and payload state visible; heavier vector overlays remain
optional future extensions.

### 2. Fast Headless Simulation
Run headlessly and generate paper figures:

```powershell
python run/run_theory_suite.py --mode bregman --coriolis c1 --duration 30 --save-figures
```

The results and figures will be saved in `results/timestamped/<timestamp>/bregman_c1/`.
Use `--timestamped-save` to save under `results/timestamped/<timestamp>/bregman_c1/` instead.

---

## Batch Comparisons

Run all 6 controller variants (`nominal`, `euclidean`, `bregman` $\times$ `c1`, `c2`) with the 10-second payload drop:

```powershell
# Parallel execution across CPU cores
python run/run_batch.py

# Serial execution (single process)
python run/run_batch.py --serial --duration 10.0
```

---

## Inspecting Recorded Trajectories

Visualize reference trajectories from `trajectories/processed/`:

```powershell
python run/plot_trajectories.py --replay-id lemniscate_01_auto
```

---

## Running Automated Tests

Run the full pytest test suite:

```powershell
pytest tests/ -v
```

All 59+ unit and integration tests should pass.
