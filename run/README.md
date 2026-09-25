# Run Scripts

All scripts in this directory are Python CLI entry points. Run them from the repository root within the `agc` Conda environment.

```powershell
conda activate agc
```

---

## Script Overview

### 1. `run_theory_suite.py`
Simulates a single UAV tracking experiment in PyBullet.
- Releases the 0.75 kg payload at 10.0 s (when duration $\ge 10$ s).
- Saves results to `results/timestamped/<timestamp>/<mode>_<coriolis>/` with `run.npz` and `metadata.json`.

```powershell
# Headless run (fast)
python run/run_theory_suite.py --mode bregman --coriolis lc --duration 30

# Interactive 3D PyBullet GUI with trajectory trail and camera tracking
python run/run_theory_suite.py --mode bregman --coriolis lc --gui --speed 1.0

# Export paper figures
python run/run_theory_suite.py --mode bregman --coriolis lc --save-figures
```

**Options:**
- `--replay-id`: Trajectory benchmark ID (default: `lemniscate_01_auto`).
- `--mode`: Controller mode: `nominal`, `euclidean`, or `bregman`.
- `--coriolis`: Coriolis factorization: `lc` (Levi-Civita) or `rb` (coadjoint).
- `--duration`: Flight duration in seconds (default: 30.0 s).
- `--gui`: Launch 3D PyBullet GUI.
- `--speed`: GUI playback speed multiplier (default: 1.0).
- `--no-pacing`: Run as fast as possible without real-time wall-clock sleep.
- `--save-figures`: Generate and save tracking/estimation figures.
- `--output-dir`: Custom output directory.
- `--timestamped-save`: Save under `results/timestamped/<timestamp>/<mode>_<coriolis>/` instead of the default in-place directory.

---

### 2. `run_batch.py`
Runs the full 6-variant comparison matrix (`nominal`, `euclidean`, `bregman` $\times$ `lc`, `rb`) with payload drop, saving run data, `manifest.json`, and comparison figures.

```powershell
# Default batch (parallel execution)
python run/run_batch.py

# Serial execution (single process)
python run/run_batch.py --serial --duration 10.0
```

**Options:**
- `--duration`: Flight duration in seconds (default: 30.0 s).
- `--replay-id`: Benchmark trajectory ID (default: `lemniscate_01_auto`).
- `--serial`: Execute scenarios sequentially in a single process.
- `--parallel`: Execute scenarios across multi-core worker processes (default).
- `--output-dir`: Custom output directory for the suite.
- `--timestamped-save`: Save under `results/timestamped/<timestamp>/` instead of the default in-place directory.
- `--no-figures`: Skip figure generation.

---

### 3. `optimize_gains.py`
Performs staged block-coordinate gain optimization (Particle Swarm or Differential Evolution) over tracking gains ($K_R, K_\xi$), damping/metric gains ($\Lambda, k_s, k_d$), and adaptation gains ($\gamma_E, \gamma_B$).

```powershell
# Optimize Bregman LC gains using hierarchical 7-stage schedule
python run/optimize_gains.py --mode bregman --coriolis lc --schedule hierarchical

# Optimize Euclidean RB using Differential Evolution with Nelder-Mead polish
python run/optimize_gains.py --mode euclidean --coriolis rb --method de --polish
```

**Options:**
- `--mode`: `nominal`, `euclidean`, `bregman`, or `all`.
- `--coriolis`: `lc`, `rb`, or `all`.
- `--schedule`: `hierarchical` (7 stages: $K \to \Lambda,k_s,k_d \to \Lambda \to k_s,k_d \to \gamma \to \text{all}$) or `classic` (4 stages).
- `--method`: `pso` (Particle Swarm) or `de` (Differential Evolution).
- `--polish`: Run Nelder-Mead simplex polishing on final `all` stage.
- `--duration`: Evaluation flight duration in seconds (default: 30.0 s).
- `--replay-id`: Trajectory benchmark ID (default: `lemniscate_01_auto`).
- `--swarm-size`: Swarm size / population multiplier (default: 50).
- `--max-iter`: Maximum iterations per stage (default: 20).
- `--max-stall`: Maximum iterations without improvement (default: 10).
- `--seed`: Random seed for reproducible search.
- `--no-parallel`: Disable multi-core particle evaluations.
- `--no-promote`: Do not update the gain registries if incumbent improves.
- `--output-dir`: Custom output directory for checkpoints and logs.
- `--timestamped-save`: Save checkpoints under `results/optimization/timestamped/<timestamp>/` instead of the default in-place directory.

---

### 4. `replay_run.py`
Visualizes a previously saved simulation run (`run.npz`) in the interactive 3D PyBullet GUI.

```powershell
python run/replay_run.py results/inplace/bregman_lc --speed 1.5
```

**Options:**
- `run_dir`: Positional argument; path to directory containing `run.npz`.
- `--speed`: Playback speed multiplier (default: 1.0).
- `--frame-stride`: Simulation step stride for rendering (default: 5).

---

### 5. `plot_trajectories.py`
Inspects reference trajectory geometry and time profiles directly from `trajectories/processed/*.mat`.

```powershell
python run/plot_trajectories.py --replay-id lemniscate_01_auto
```

**Options:**
- `--replay-id`: Benchmark trajectory ID (e.g. `lemniscate_01_auto`, `ellipse_01_auto`, `RATM_01_auto`).
- `--output`: File path to save the generated plot image.

---

### 6. `generate_paper_figures.py`
Regenerates publication-ready figures (attitude/position error, parameter estimates, C1 vs C2 comparisons, Euclidean vs Bregman comparisons) from a saved run or suite directory.

```powershell
python run/generate_paper_figures.py results/timestamped/<timestamp>
```
