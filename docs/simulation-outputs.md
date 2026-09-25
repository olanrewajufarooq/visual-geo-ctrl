# Simulation Outputs

Simulation runs and batch comparison suites write structured results under `results/inplace/` by default, or `results/timestamped/<timestamp>/` when `--timestamped-save` is used. Gain optimization checkpoints use the separate `results/optimization/best-gain/` and `results/optimization/timestamped/<timestamp>/` trees.

## Directory Hierarchy

### Single Runs (`run_theory_suite.py`)
```text
results/timestamped/<timestamp>/
`-- <mode>_<coriolis>/
    |-- run.npz
    |-- metadata.json
    `-- figures/
        |-- tracking_error.png
        `-- parameter_estimates.png
```

### Batch Suites (`run_batch.py`)
```text
results/inplace/
`--
    |-- manifest.json
    |-- nominal_lc/
    |   |-- run.npz
    |   `-- metadata.json
    |-- nominal_rb/
    |   |-- run.npz
    |   `-- metadata.json
    |-- euclidean_lc/
    |   |-- run.npz
    |   `-- metadata.json
    |-- euclidean_rb/
    |   |-- run.npz
    |   `-- metadata.json
    |-- bregman_lc/
    |   |-- run.npz
    |   `-- metadata.json
    |-- bregman_rb/
    |   |-- run.npz
    |   `-- metadata.json
    `-- figures/
        |-- lc_vs_rb_tracking_error.png
        |-- lc_vs_rb_parameter_error.png
        |-- euclidean_vs_bregman_tracking_error.png
        `-- euclidean_vs_bregman_parameter_error.png
```

---

## Artifact Schemas

### 1. `run.npz` (NumPy Compressed Archive)
Contains dense time-series arrays logged at the 500 Hz physics simulation rate:

- `t`: $N$-vector of simulation timestamps $[0, t_{\text{end}}]$.
- `R`: $(N, 3, 3)$ vehicle rotation matrices in $\mathrm{SO}(3)$.
- `p`: $(N, 3)$ vehicle position in world frame $\mathbb{R}^3$.
- `V`: $(N, 6)$ body-frame generalized twist $[\Omega^T, v^T]^T$.
- `R_d`, `p_d`, `V_d`, `V_d_dot`: Desired trajectory state arrays.
- `tau`: $(N, 6)$ body-frame control wrench applied to the plant.
- `eta_R`: $(N, 3)$ attitude tracking error vectors.
- `eta_xi`: $(N, 3)$ position tracking error vectors.
- `s`: $(N, 6)$ sliding surface vector.
- `hat_pi`: $(N, 10)$ estimated inertial parameters.
- `hat_J`: $(N, 4, 4)$ estimated pseudo-inertia matrix (Bregman mode).

### 2. `metadata.json`
Schema version 1.0 JSON document storing simulation parameters, metrics, and completion state:
```json
{
  "schema_version": "1.0",
  "created_at": "2026-09-21T23:00:00.000000",
  "mode": "bregman",
  "coriolis": "lc",
  "replay_id": "lemniscate_01_auto",
  "timing": {
    "duration": 30.0,
    "dt_plant": 0.002,
    "dt_control": 0.02,
    "dt_adaptation": 0.01,
    "release_time": 10.0
  },
  "metrics": {
    "pos_rmse": 0.0215,
    "att_rmse": 0.0042,
    "vel_rmse": 0.0381,
    "mass_rmse": 0.0120,
    "com_rmse": 0.0018,
    "inertia_rmse": 0.0035,
    "wrench_rmse": 14.82
  },
  "completed": true,
  "failure": null
}
```

### 3. `manifest.json`
Batch suite manifest tracking execution mode (`serial` vs `parallel`), all variant member results, summary performance table, and comparison figure locations.
