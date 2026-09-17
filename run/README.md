# Run scripts

Every `.m` file here is a MATLAB script. Edit its settings at the top, then run it from the repository root after `startup`.

- `run_theory_suite.m` runs nominal, Euclidean, and Bregman variants for both `C1` and `C2`; each result is saved under `results/<timestamp>/`.
- `run_batch.m` runs the same matrix with parallel workers by default, saves every successful run, and exports figures after a complete suite.
- `optimize_gains.m` uses Global Optimization Toolbox to tune all controller gains and promotes each winner into `config/optimized_gains.m`. Set either selector to `''` for all supported values, or pass a cell array such as `{'bregman','euclidean'}`.
- `replay_run.m` visualizes a saved result and can export an MP4.
- `plot_trajectories.m` plots one recorded artifact from `trajectories/processed/`; it does not run a simulation.
- `generate_paper_figures.m` exports the C1/C2 and Euclidean/Bregman comparison figures from one saved suite, or the newest complete suite when given `results`.
