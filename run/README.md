# Run scripts

Every `.m` file here is a MATLAB script. Edit its settings at the top, then run it from the repository root after `startup`.

- `run_theory_suite.m` runs nominal, Euclidean, and Bregman variants for both `C1` and `C2` over a 30 s replay segment; each result is saved under `results/<timestamp>/`.
- `run_batch.m` runs the same matrix with parallel workers by default, saves every successful run, and exports figures after a complete suite. It writes paired comparisons to `results/<timestamp>/comparisons/{nominal,euclidean,bregman,euclidean_v_bregman,performance}/` and standalone diagnostics beside each `run.mat` at `results/<timestamp>/<mode>_<coriolis>/figures/`.
- `optimize_gains.m` uses Global Optimization Toolbox to tune all controller gains and promotes each winner into `config/optimized_gains.m`. Set either selector to `''` for all supported values, or pass a cell array such as `{'bregman','euclidean'}`.
- `replay_run.m` visualizes a saved result and can export an MP4.
- `plot_trajectories.m` plots one recorded artifact from `trajectories/processed/`; it does not run a simulation.
- `generate_paper_figures.m` re-exports the paired comparisons and every standalone diagnostic from one saved suite, or the newest complete suite when given `results`.
