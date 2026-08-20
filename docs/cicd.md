# CI/CD

The repository includes a GitHub Actions workflow at `.github/workflows/release-results.yml` for headless MATLAB result generation and release packaging.

## Trigger

The workflow runs on tag pushes matching:

```text
v*
```

## What the Workflow Does

1. Checks out the repository.
2. Sets up MATLAB through `matlab-actions/setup-matlab`.
3. Runs `ci_release` headlessly with software OpenGL and invisible figures.
4. Verifies that the `results/` folder was generated.
5. Builds release notes from aggregated `command_window.txt` files.
6. Packages the `results/` directory as a zip artifact.
7. Uploads the artifact and publishes a GitHub release.

## MATLAB Command Used

```text
set(0,'DefaultFigureVisible','off'); addpath('run'); ci_release;
```

## Required Secret

The workflow expects:

- `MATLAB_TOKEN` mapped into `MLM_LICENSE_TOKEN`

## Release Output

Generated release assets include:

- `results-<tag>.zip`
- GitHub release notes built from aggregated simulation logs

## Operational Notes

- The workflow is currently centered on the adaptive release scenario in `run/ci_release.m`.
- If you change output folder structure or log naming, update the release note generation step so it still finds the aggregated `command_window.txt` files.
- For local validation, run `ci_release` directly in MATLAB before pushing a release tag.
