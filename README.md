# Adaptive Geometric Control

MATLAB reference implementation of the paper's fully actuated UAV tracking controller on `SE(3)`. Robotics System Toolbox supplies the floating-base `rigidBodyTree` plant; controller, regressor, and adaptation code live in a small equation-oriented package.

## Run

Open the repository in MATLAB and run `startup`, then use one of the scripts in `run/`:

- `run_theory_suite` — nominal, Euclidean, and Bregman controllers with `C1` and `C2`.
- `run_batch` — independent variants through the parallel batch runner.
- `optimize_gains` — explicit replay tracking/effort optimization.
- `replay_run` — post-process a saved `results/v2/...` run as a 3-D trajectory animation.
- `plot_trajectories` — inspect a recorded, preprocessed replay trajectory.
- `generate_paper_figures` — export static tracking and adaptation figures from a saved theory-suite directory.

Edit the settings at the top of each script before running it.

## Layout

- `src/+agc/+math` — SE(3), generalized inertia, pseudo-inertia, SPD utilities.
- `src/+agc/+paper` — tracking errors, Coriolis factorizations, regressor, controller, and adaptation.
- `src/+agc/+plant` — Robotics System Toolbox floating-body wrapper.
- `src/+agc/+sim`, `+batch`, `+opt`, `+viz`, `+io` — scenario execution and workflows.
- `trajectories/` — preserved recorded replay artifacts.
- `tests/` — MATLAB unit and integration tests.

Run the test suite with `runtests('tests')`.
