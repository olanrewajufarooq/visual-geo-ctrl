# Troubleshooting

## MATLAB Path Issues

Symptoms:

- `fth.*` classes or functions are not found
- Run scripts fail immediately on startup

Checks:

```matlab
startup
which fth.sim.SimRunner
which fth.sim.Config
```

If those lookups fail, make sure you started MATLAB in the repository root or explicitly `cd` into it before calling `startup`.

## Visualization Problems

Common causes:

- Missing Robotics System Toolbox
- Graphics driver or renderer issues
- Overly aggressive live-update settings during long runs

Mitigations:

```matlab
cfg.enableLiveView(false);
cfg.setLiveUrdfEmbedding(false);
cfg.setLiveUpdateRate(500);
```

If URDF rendering is unavailable, prefer lighter plotting paths and summary figures.

## Numerical or Stability Issues

Symptoms:

- Divergent trajectories
- Unstable adaptive estimates
- Integration errors or unrealistic motion

Checks:

- Reduce `dt`, `controlDt`, or `adaptationDt`
- Verify payload mass, position, and drop timing
- Re-check controller gains and adaptation gains
- Start from a nominal scenario before enabling adaptation

## Batch Run Performance

For large sweeps:

- Disable live plotting
- Use `plotMode = 'summary'` or `plotMode = 'none'`
- Save `sim_data.mat` only when you actually need it
- Use `parallelRuns = true` only in environments where MATLAB parallel execution is configured

## CI/Release Failures

If the GitHub Actions release job fails:

- Confirm `MATLAB_TOKEN` is configured in repository secrets
- Run `ci_release` locally first
- Check whether result-folder naming or log generation changed in a way that breaks the workflow log collection step
