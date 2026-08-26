# Run scripts

The MATLAB files in this directory are grouped by purpose. Run them from the
repository root after the project paths have been initialized by the script's
`startup` call.

## `run_nominal_*`

Nominal-controller demonstrations and comparisons. These scripts run the
controller without online adaptation and are useful for checking baseline
tracking behavior and comparing Coriolis-model choices.

Examples:

- `run_nominal_demo.m`
- `run_nominal_coriolis_comparison.m`

## `run_adaptive_*`

Adaptive-controller demonstrations, reproductions, and comparisons. These
scripts configure Euclidean or Bregman adaptation, payload conditions, and
replay trajectories for evaluating adaptive behavior.

Examples:

- `run_adaptive_demo.m`
- `run_adaptive_gain_comparison.m`
- `run_adaptive_coriolis_comparison.m`
- `run_adaptive_reproduce.m`

## `plot_*`

Plotting and replay-analysis scripts. They load or generate trajectory data
and produce visualizations; they do not perform gain optimization.

Examples:

- `plot_analytic_trajectories.m`
- `plot_replay_trajectories.m`

## `optimize_*`

Gain-tuning workflows using MATLAB's Global Optimization Toolbox. The general
workflow is `optimize_gains.m`, which optimizes the full controller/adaptation
gain vector for the selected scenarios and writes reports under
`results/tuning/`.

The Gamma-only workflows reuse completed general-run gains for fixed `Kp`,
`Kd`, and `lambda` values:

- `optimize_bregmann_gamma_gain.m` optimizes scalar Bregman Gamma.
- `optimize_euclidean_gamma_gains.m` optimizes the ten Euclidean Gamma values.

The Gamma scripts configure their options through `opt` and delegate the
workflow internally. By default, the newest matching general-run report is
selected automatically. See `docs/gain_tuning_data_output.md` for report
files, search-bound overrides, and direct fixed-gain configuration.

## Common notes

- Use `clearCache = true` when a fresh optimization is required; otherwise a
  matching incomplete run can resume from its checkpoint.
- Optimization scripts require Global Optimization Toolbox. Parallel runs
  also require Parallel Computing Toolbox.
- Generated simulation and optimization data is written below `results/`,
  which is intentionally excluded from version control.
