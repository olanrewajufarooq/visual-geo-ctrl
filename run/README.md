# Run scripts

Every `.m` file here is a MATLAB script. Edit its settings at the top, then run it from the repository root after `startup`.

`run_batch.m` and `run_theory_suite.m` default to `inplaceSave = true`, writing to `results/inplace/`. They overwrite only the successful scenario subfolders selected by that run, such as `results/inplace/nominal_c2/`; other in-place scenarios and `results/inplace/comparisons/` remain untouched. In-place runs export only standalone diagnostics. Set `inplaceSave = false` to create a new timestamped suite under `results/<timestamp>/`, including comparisons.

- `run_theory_suite.m` runs a selectable payload-drop subset over a 30 s replay segment. Every case begins with an exact loaded UAV model, releases the fixed 0.75 kg payload at 10 s, and is saved under `results/<timestamp>/`. Set `generateFigures` to plot successful cases and `saveReplay3D` to write an MP4 beside each successful `run.mat`.
- `run_batch.m` runs the six-case payload-drop matrix with parallel workers by default and saves every successful run. It exports standalone diagnostics for every successful run plus only comparisons whose required members are available. Set `generateFigures` and `saveReplay3D` at the top of the script. Comparisons are written to `results/<timestamp>/comparisons/{nominal,euclidean,bregman,euclidean_v_bregman,performance}/`; standalone diagnostics stay beside each `run.mat` at `results/<timestamp>/<mode>_<coriolis>/figures/`.
- `optimize_gains.m` uses Global Optimization Toolbox for staged derivative-free block-coordinate optimization. It runs an all-gain particle swarm, a non-adaptive block swarm, an adaptive block search, then a final all-gain polish; Bregman uses a saved scalar `gammaB` log-grid profile. Its common objective prioritizes position/attitude tracking and adds lower-weight normalized inertial-estimation RMSE plus wrench effort. Each stage retains the best full-horizon feasible incumbent and may update only its matching entry in `config/optimized_gains.m`. Set either selector to `''` for all supported values, or pass a cell array such as `{'bregman','euclidean'}`.
- `replay_run.m` visualizes a saved result and can export an MP4.
- `plot_trajectories.m` plots one recorded artifact from `trajectories/processed/`; it does not run a simulation.
- `generate_paper_figures.m` re-exports available comparisons and every standalone diagnostic from one saved suite, or the newest suite containing saved runs when given `results`.

## Optimization reference

The staged method uses block-coordinate terminology in the sense of P. Tseng, [“Convergence of a Block Coordinate Descent Method for Nondifferentiable Minimization”](https://www.mit.edu/~dimitrib/PTseng/papers/archive/bcr_jota.pdf), *Journal of Optimization Theory and Applications*, 109(3), 475–494, 2001. The particle-swarm and grid stages are approximate derivative-free block solves; this implementation does not claim the convergence guarantees of exact block-coordinate descent.
