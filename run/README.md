# Run Scripts

All scripts in this directory are Python CLI entry points. Run them from the repository root within the `agc` Conda environment.

```powershell
conda activate agc
```

---

## Script Overview

### 1. `run_sim.py`
Simulates a single UAV tracking experiment in PyBullet.
- Nominal mode runs bare by default; Euclidean and Bregman modes enable the 0.75 kg payload-release scenario by default (release at 10.0 s when duration $\ge 10$ s).
- Use `--payload` to enable payload release for nominal mode or `--no-payload` to disable it for an adaptive mode.
- Explicit payload overrides that differ from the mode default use a `_payload` or `_bare` output-directory suffix to keep the results separate. Default runs retain `<mode>_<coriolis>`.
- Saves results under `results/timestamped/<timestamp>/` or `results/inplace/` with `run.npz` and `metadata.json`.

```powershell
# Headless run (fast)
python run/run_sim.py --mode bregman --coriolis lc --duration 30

# Nominal, bare vehicle by default; optionally run nominal with payload release
python run/run_sim.py --mode nominal --coriolis lc --duration 30 --payload

# Adaptive estimator without payload release
python run/run_sim.py --mode bregman --coriolis lc --duration 30 --no-payload

# Interactive 3D PyBullet GUI with trajectory trail and camera tracking
python run/run_sim.py --mode bregman --coriolis lc --gui --speed 1.0

# Export paper figures
python run/run_sim.py --mode bregman --coriolis lc --save-figures
```

**Options:**
- `--replay-id`: Trajectory benchmark ID (default: `lemniscate_01_auto`).
- `--mode`: Controller mode: `nominal`, `euclidean`, or `bregman`.
- `--coriolis`: Coriolis factorization: `lc` (Levi-Civita) or `rb` (coadjoint).
- `--duration`: Flight duration in seconds (default: 30.0 s).
- `--payload` / `--no-payload`: Override the mode-based payload default.
- `--gui`: Launch 3D PyBullet GUI.
- `--speed`: GUI playback speed multiplier (default: 1.0).
- `--no-pacing`: Run as fast as possible without real-time wall-clock sleep.
- `--save-figures`: Generate and save tracking/estimation figures.
- `--output-dir`: Custom output directory.
- `--timestamped-save`: Save under `results/timestamped/<timestamp>/<mode>_<coriolis>/` instead of the default in-place directory.

---

### 2. `run_batch_sim.py`
Runs the full 6-variant comparison matrix (`nominal`, `euclidean`, `bregman` $\times$ `lc`, `rb`) with payload drop, saving run data, `manifest.json`, and comparison figures.

```powershell
# Default batch (parallel execution)
python run/run_batch_sim.py

# Serial execution (single process)
python run/run_batch_sim.py --serial --duration 10.0
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

### 3. `run_paper_sim.py`
Runs the official publication experiment suite and generates multi-panel LaTeX-ready publication figures in `results/papers/figures/`.

```powershell
# Run all publication studies and export all figures
python run/run_paper_sim.py all

# Run specific paper studies
python run/run_paper_sim.py adaptive-drop
python run/run_paper_sim.py nominal-connection
python run/run_paper_sim.py nominal-reaching
python run/run_paper_sim.py connection-realizations
python run/run_paper_sim.py connection-sensitivity
```

**Options:**
- `command`: Study to run: `all`, `adaptive-drop`, `nominal-connection`, `nominal-reaching`, `connection-realizations`, `connection-sensitivity` (default: `all`).
- `--duration`: Flight duration in seconds (default: 30.0 s).
- `--reuse-cache`: Reuse raw run cache when source code fingerprint matches.
- `--output-dir`: Output directory for figures and summaries (default: `results/papers`).
- `--raw-output-dir`: Output directory for raw simulation runs (default: `results/paper-runs`).

---

### 4. `optimize_gains.py`
Performs staged block-coordinate gain optimization with Particle Swarm Optimization or Differential Evolution over tracking gains ($K_R, K_\xi$), damping/metric gains ($\Lambda, k_s, k_d$), and adaptation gains ($\gamma_E, \gamma_B$).

```powershell
# Optimize all three modes independently for both LC and RB
python run/optimize_gains.py --mode all --coriolis all --schedule hierarchical

# Optimize nominal tracking gains only
python run/optimize_gains.py --mode nominal --swarm-size 30 --max-iter 40

# Optimize Bregman gains for the RB factorization with DE and optional polish
python run/optimize_gains.py --mode bregman --coriolis rb --method de --polish
```

**Options:**
- `--mode`: `nominal`, `euclidean`, `bregman`, or `all` (default: `all`).
- `--coriolis`: `lc`, `rb`, or `all` (default: `all`). Each mode/factorization pair is optimized and promoted independently.
- `--schedule`: `hierarchical` or `classic`. Hierarchical uses six stages for nominal and seven for adaptive modes; classic uses one nominal stage and four adaptive stages.
- `--method`: `pso` (default) or `de`; `--polish` optionally applies Nelder–Mead on the final all-gains stage.
- `--duration`: Evaluation flight duration in seconds (default: 30.0 s).
- `--train-replay-ids`: Comma-separated replay IDs (default: `lemniscate_02_auto,lemniscate_03_auto,lemniscate_04_auto`).
- `--training-payload-profiles`: Comma-separated named payload profiles (default: `flat_light,tall_heavy`). All training conditions include payload release.
- `--swarm-size`: Swarm size for PSO / population multiplier for DE (default: 20).
- `--max-iter`: Maximum iterations per stage (default: 50).
- `--max-stall`: Maximum iterations without improvement (default: 10).
- `--seed`: Random seed for reproducible search.
- `--no-parallel`: Disable multi-core particle evaluations.
- `--no-promote`: Do not update the gain registries if incumbent improves.
- `--output-dir`: Custom output directory for checkpoints and logs.
- `--timestamped-save`: Save checkpoints under `results/optimization/timestamped/<timestamp>/` instead of the default in-place directory.

---

### 5. `replay_run.py`
Visualizes a previously saved simulation run (`run.npz`) in the interactive 3D PyBullet GUI.

```powershell
python run/replay_run.py results/inplace/bregman_lc --speed 1.5
```

**Options:**
- `run_dir`: Positional argument; path to directory containing `run.npz`.
- `--speed`: Playback speed multiplier (default: 1.0).
- `--frame-stride`: Simulation step stride for rendering (default: 5).

---

### 6. `plot_trajectories.py`
Inspects reference trajectory geometry and time profiles directly from `trajectories/processed/*.mat`.

```powershell
python run/plot_trajectories.py --replay-id lemniscate_01_auto
```

**Options:**
- `--replay-id`: Benchmark trajectory ID (e.g. `lemniscate_01_auto`, `ellipse_01_auto`, `RATM_01_auto`).
- `--output`: File path to save the generated plot image.

---

### 7. `plot_simulation_setup.py`
Generates publication-quality 3D simulation-setup figures (arena, gates, trajectory, vehicle, and IEEE composite panels) in both PNG and 300 DPI PDF formats.

```powershell
python run/plot_simulation_setup.py
```
