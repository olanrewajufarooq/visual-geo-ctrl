# Project Structure

## Repository Layout

```text
adaptive-geo-ctrl-pybullet/
|-- assets/
|   `-- hexacopter_description/urdf/
|-- docs/
|   |-- architecture.md
|   |-- batch-simulations.md
|   |-- cicd.md
|   |-- coding-conventions.md
|   |-- configuration.md
|   |-- customization.md
|   |-- features.md
|   |-- gain-tuning.md
|   |-- gain_tuning_data_output.md
|   |-- getting-started.md
|   |-- project-structure.md
|   |-- pybullet_matlab_differences.md
|   |-- simulation-outputs.md
|   |-- troubleshooting.md
|   `-- README.md
|-- run/
|   |-- optimize_gains.py
|   |-- plot_simulation_setup.py
|   |-- plot_trajectories.py
|   |-- replay_run.py
|   |-- run_batch.py
|   |-- run_paper_experiments.py
|   |-- run_theory_suite.py
|   `-- README.md
|-- src/agc/
|   |-- __init__.py
|   |-- batch/
|   |   |-- __init__.py
|   |   `-- run_batch.py
|   |-- config/
|   |   |-- __init__.py
|   |   |-- manual_gains.py
|   |   `-- optimized_gains.py
|   |-- io/
|   |   |-- __init__.py
|   |   `-- persistence.py
|   |-- math/
|   |   |-- __init__.py
|   |   |-- inertia.py
|   |   `-- se3.py
|   |-- opt/
|   |   |-- __init__.py
|   |   |-- bounds.py
|   |   |-- bregman_profile.py
|   |   |-- encoding.py
|   |   |-- objective.py
|   |   |-- pso.py
|   |   `-- staged_optimizer.py
|   |-- paper/
|   |   |-- __init__.py
|   |   |-- adaptation.py
|   |   |-- controller.py
|   |   |-- coriolis.py
|   |   |-- errors.py
|   |   `-- regressor.py
|   |-- plant/
|   |   |-- __init__.py
|   |   |-- compound_pi.py
|   |   |-- drone_urdf.py
|   |   |-- pybullet_plant.py
|   |   `-- suppress.py
|   |-- sim/
|   |   |-- __init__.py
|   |   |-- default_scenario.py
|   |   |-- metrics.py
|   |   |-- replay_trajectory.py
|   |   |-- run_scenario.py
|   |   `-- validation.py
|   `-- viz/
|       |-- __init__.py
|       |-- paper_figures.py
|       `-- pybullet_viz.py
|-- tests/
|   |-- conftest.py
|   |-- test_batch.py
|   |-- test_gain_registry.py
|   |-- test_manual_gains.py
|   |-- test_math.py
|   |-- test_optimization.py
|   |-- test_paper_core.py
|   |-- test_persistence.py
|   |-- test_pybullet_plant.py
|   |-- test_pybullet_viz.py
|   |-- test_scenario_runner.py
|   |-- test_trajectory_processing.py
|   |-- test_validation.py
|   `-- test_workflow.py
|-- trajectories/
|   |-- download_ratm_trajectories.py
|   |-- process_trajectories.py
|   |-- processed/
|   |   |-- ellipse_01_auto.npz
|   |   |-- lemniscate_01_auto.npz
|   |   |-- manifest.json
|   |   `-- RATM_01_auto.npz
|   |-- replay_scripts/
|   |   |-- __init__.py
|   |   |-- replay_kinematics.py
|   |   |-- replay_processing_core.py
|   |   |-- replay_processor.py
|   |   |-- replay_traj.py
|   |   |-- replay_wnoj_smoother.py
|   |   `-- write_replay_artifact.py
|   |-- POSTPROCESSING.md
|   `-- README.md
|-- .github/workflows/
|   |-- ci.yml
|   `-- release-results.yml
|-- environment.yml
|-- pyproject.toml
|-- AGENT.md
`-- README.md
```

## Package Roles

| Package | Purpose |
| --- | --- |
| `agc.math` | $\mathrm{SE}(3)$ Lie group/algebra utilities (`se3.py`), pseudo-inertia conversions, and SPD tests (`inertia.py`). |
| `agc.paper` | Paper equations: Coriolis factorizations (`c1`, `c2`), tracking errors, regressor $Y$, and adaptation laws. |
| `agc.plant` | PyBullet floating-base multi-body vehicle plant, body-wrench application, and payload detachment. |
| `agc.sim` | Multi-rate simulation runner (500 Hz / 100 Hz / 50 Hz), continuous trajectory replay, metrics, and scenario validation. |
| `agc.config` | Hand-tuned manual gains (`manual_gains.py`) and tuned optimized gain registry (`optimized_gains.py`). |
| `config` | Root convenience package re-exporting gain registries. Synchronized automatically on gain promotion. |
| `agc.opt` | Staged block-coordinate gain optimization (hierarchical and classic schedules), PSO, DE, and Nelder-Mead polishing. |
| `agc.io` | Persistence utilities for `.npz` simulation runs, `metadata.json`, and `manifest.json`. |
| `agc.batch` | Multi-variant batch simulation engine with process isolation and error recovery. |
| `agc.viz` | Headless publication figures (`total-sim` and `from-drop` views) and interactive 3D visualizer (`pybullet_viz.py`). |
