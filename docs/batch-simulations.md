# Batch Simulations

The framework provides batch execution to simulate and compare all controller and Coriolis variants over benchmark trajectories with payload detachment.

Batch simulation is orchestrated by `src/agc/batch/run_batch.py` and executed via the CLI runner `run/run_batch.py`.

## Running Batch Simulations

### Parallel Execution (Recommended)
By default, `run/run_batch.py` launches worker processes to execute scenarios concurrently, maintaining independent PyBullet physics client contexts:

```powershell
python run/run_batch.py --duration 30.0 --replay-id lemniscate_01_auto
```

### Serial Execution
For single-threaded debugging or memory-constrained environments:

```powershell
python run/run_batch.py --serial --duration 10.0
```

## The 6-Variant Matrix

The batch suite executes all combinations of:
- **Modes**: `nominal`, `euclidean`, `bregman`
- **Coriolis Factorizations**: `c1` (Levi-Civita), `c2` (coadjoint)

Each run simulates the vehicle carrying a 0.75 kg payload, followed by dynamic detachment at $t = 10.0$ s.

## Output Structure

By default, results are stored in `results/timestamped/<timestamp>/` containing:
- `manifest.json`: Suite metadata, execution mode, variant status, and summary comparison table.
- `<mode>_<coriolis>/run.npz`: Complete recorded states, errors, wrenches, and estimates.
- `<mode>_<coriolis>/metadata.json`: Run parameters, timing, tracking RMSE metrics, and failure status.
- `figures/`: Publication-quality comparison plots:
  - `c1_vs_c2_tracking_error.png`
  - `c1_vs_c2_parameter_error.png`
  - `euclidean_vs_bregman_tracking_error.png`
  - `euclidean_vs_bregman_parameter_error.png`
  - Per-variant standalone state and error trajectories.

Use `python run/run_batch.py --inplace-save` to reuse `results/inplace/` instead.

## Failure Isolation & Diagnostics

If an individual scenario encounters a numerical failure or instability:
- The worker isolates the error without terminating the batch suite.
- All finite prefix samples prior to the failure are persisted to `run.npz`.
- Failure details (exception message, failure time, step index) are recorded in `metadata.json`.
- Performance comparisons gracefully exclude failed members while retaining diagnostic plots.
