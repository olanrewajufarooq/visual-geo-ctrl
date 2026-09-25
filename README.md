# Adaptive Geometric Control (Python & PyBullet)

Python reference implementation of the paper's fully actuated UAV tracking controller on $\mathrm{SE}(3)$ with **PyBullet** physics simulation.

Replaces MATLAB's Robotics System Toolbox forward dynamics with a native floating-base PyBullet multi-body simulation plant with real-time 3D visual rendering, camera tracking, and physical payload detachment.

---

## 1. Setup

Activate the dedicated Conda environment created according to the KFUPM reference:

```powershell
conda activate agc
```

To re-create or verify the environment:
```powershell
conda env create -f environment.yml
```

---

## 2. Run Simulation

### Run Single Scenario (Theory Suite)

Run headless:
```powershell
python run/run_theory_suite.py --mode bregman --coriolis lc --duration 30
```

Run with interactive **3D PyBullet GUI** (camera tracking drone and trajectory visual trail):
```powershell
python run/run_theory_suite.py --mode bregman --coriolis lc --gui
```

Controller options:
- `--mode`: `nominal`, `euclidean`, or `bregman`
- `--coriolis`: `lc` (Levi-Civita connection) or `rb` (coadjoint factorization)
- `--duration`: flight duration in seconds (default: 30 s)
- `--gui`: opens 3D PyBullet visualizer window

---

### Run 6-Variant Comparison Batch

```powershell
python run/run_batch.py
```
Simulates all 6 variants (`nominal`, `euclidean`, `bregman` $\times$ `c1`, `c2`) with the 10-second payload drop, and writes results and publication figures to `results/inplace/` by default. Pass `--timestamped-save` to use `results/timestamped/<timestamp>/`.

---

### Inspect Trajectories

```powershell
python run/plot_trajectories.py
```

---

## 3. Run Automated Tests

Run the comprehensive pytest suite:
```powershell
pytest tests/ -v
```

---

## 4. Architecture

- `src/agc/math`: $\mathrm{SE}(3)$ and $\mathrm{SO}(3)$ Lie group and Lie algebra operations, generalized 6D inertia, pseudo-inertia, and SPD verification.
- `src/agc/paper`: Metric-compatible Coriolis factorizations (`c1`, `c2`), tracking error covectors, 6x10 regressor $Y$, and discrete adaptation laws.
- `src/agc/plant`: PyBullet floating-base multi-body vehicle plant with link-frame body wrench application and dynamic payload detachment.
- `src/agc/sim`: Multi-rate simulation loop (500 Hz physics, 100 Hz adaptation, 50 Hz control), continuous $C^1$ trajectory sampler, and tracking RMSE metrics.
- `src/agc/viz`: Publication-quality Matplotlib figures.
