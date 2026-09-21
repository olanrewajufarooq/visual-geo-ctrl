# Project Structure

## Repository Layout

```text
adaptive-geo-ctrl-pybullet/
|-- assets/
|   `-- hexacopter_description/urdf/
|-- config/
|   |-- manual_gains.py
|   `-- optimized_gains.py
|-- docs/
|   |-- architecture.md
|   |-- batch-simulations.md
|   |-- gain-tuning.md
|   |-- getting-started.md
|   |-- project-structure.md
|   |-- pybullet_matlab_differences.md
|   |-- simulation-outputs.md
|   `-- troubleshooting.md
|-- run/
|   |-- generate_paper_figures.py
|   |-- optimize_gains.py
|   |-- plot_trajectories.py
|   |-- replay_run.py
|   |-- run_batch.py
|   `-- run_theory_suite.py
|-- src/agc/
|   |-- batch/
|   |   `-- run_batch.py
|   |-- io/
|   |   `-- persistence.py
|   |-- math/
|   |   |-- adjoint.py
|   |   |-- inertia.py
|   |   `-- spd.py
|   |-- opt/
|   |   |-- bounds.py
|   |   |-- bregman_profile.py
|   |   |-- de.py
|   |   |-- encoding.py
|   |   |-- objective.py
|   |   |-- pso.py
|   |   `-- staged_optimizer.py
|   |-- paper/
|   |   |-- adaptation.py
|   |   |-- controller.py
|   |   |-- coriolis.py
|   |   |-- errors.py
|   |   `-- regressor.py
|   |-- plant/
|   |   |-- compound_pi.py
|   |   |-- drone_urdf.py
|   |   |-- pybullet_plant.py
|   |   `-- suppress.py
|   |-- sim/
|   |   |-- default_scenario.py
|   |   |-- metrics.py
|   |   |-- replay_trajectory.py
|   |   |-- run_scenario.py
|   |   `-- validation.py
|   `-- viz/
|       |-- decimate_trace.py
|       |-- paper_figures.py
|       `-- replay_3d.py
|-- tests/
|   |-- test_batch.py
|   |-- test_gain_registry.py
|   |-- test_math.py
|   |-- test_optimization.py
|   |-- test_paper_core.py
|   |-- test_persistence.py
|   |-- test_robotics_plant.py
|   |-- test_scenario_runner.py
|   |-- test_validation.py
|   `-- test_visualization.py
|-- trajectories/
|   `-- processed/
|-- environment.yml
|-- pyproject.toml
|-- AGENT.md
`-- README.md
```

## Package Roles

| Package | Purpose |
| --- | --- |
| `agc.math` | $\mathrm{SE}(3)$ Lie group/algebra utilities, adjoints, pseudo-inertia conversions, and SPD tests. |
| `agc.paper` | Paper equations: Coriolis factorizations (`c1`, `c2`), tracking errors, regressor $Y$, and adaptation laws. |
| `agc.plant` | PyBullet floating-base multi-body vehicle plant, body-wrench application, and payload detachment. |
| `agc.sim` | Multi-rate simulation runner (500 Hz / 100 Hz / 50 Hz), continuous trajectory replay, metrics, and scenario validation. |
| `agc.opt` | Staged block-coordinate gain optimization (hierarchical and classic schedules), PSO, DE, and Nelder-Mead polishing. |
| `agc.io` | Persistence utilities for `.npz` simulation runs, `metadata.json`, and `manifest.json`. |
| `agc.batch` | Multi-variant batch simulation engine with process isolation and error recovery. |
| `agc.viz` | Headless publication figure generation (C1 vs C2, Euclidean vs Bregman) and 3D replay visualizer. |
