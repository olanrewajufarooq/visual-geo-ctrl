# Gain-tuning data output

Optimization reports are written below `results/tuning/<run>/<scenario-id>/`.
The iteration number is the primary identifier. `elapsed_seconds` is recorded
as optional runtime metadata for estimating iteration duration and total cost.

## Files

- `convergence.csv`: one row for every completed PSO iteration with
  `iteration`, `best_cost`, and `elapsed_seconds`. Repeated best costs are
  retained so stall periods are visible.
- `best_improvements.csv`: rows recorded when the global best improves. It has
  `iteration`, `best_cost`, `elapsed_seconds`, `event`, named controller-gain
  columns, and physical `Gamma` columns. This is the preferred file for
  plotting gain changes that produced improvement.
- `baseline_and_best.csv`: headered `baseline` and `best` rows using the same
  named gain columns. Older generated files with two unnamed positional rows
  are legacy outputs and are not rewritten.
- `optimizer_state.mat`: MATLAB-native state including `bestX`, `bestCost`,
  `history`, `improvementHistory`, baseline/final breakdowns, options,
  checkpoint swarm, completion state, and run signature. Gamma values in
  `bestX` are log10 values; table/CSV Gamma columns are physical values
  (`10.^bestX(19:end)`).
- `run_metadata.json`: compact scenario, optimizer, completion, iteration,
  elapsed-time, and signature metadata for non-MATLAB tooling.
- `best_gains.txt`, `best_gains.m`, and `convergence.png`: human-readable,
  executable, and visual summaries of the final result.

## Gain columns

`Kp_1...Kp_6`, `Kd_1...Kd_6`, and `lambda_1...lambda_6` are controller gains.
Bregman reports contain one `Gamma_1`; Euclidean reports contain ten Gamma
columns ordered as `[m,hx,hy,hz,Ixx,Iyy,Izz,Ixy,Ixz,Iyz]`.

## Specialized Gamma runs

After a completed general run, run the corresponding script from the
repository root. With the default empty source directory, the runner selects
the newest matching general-run report under `results/tuning` and errors when
none exists:

```matlab
% Edit sourceReportDir in run/optimize_bregmann_gamma_gain.m, then:
optimize_bregmann_gamma_gain

% Edit sourceReportDir in run/optimize_euclidean_gamma_gains.m, then:
optimize_euclidean_gamma_gains
```

The Bregman script optimizes only scalar Bregman Gamma. The Euclidean script
optimizes only the ten Euclidean Gamma values. Both freeze `Kp`, `Kd`, and
`lambda` from the selected general-run scenario reports. For experiments with
user-supplied fixed gains, call `fth.opt.GammaOptimizationRunner.run` with a
`fixedGains` struct containing physical `Kp`, `Kd`, `lambda`, and `Gamma`; a
commented example is included in each script.
