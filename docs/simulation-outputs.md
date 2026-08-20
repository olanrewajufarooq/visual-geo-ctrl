# Simulation Outputs

Simulation runs write artifacts under `results/`. The exact folder name depends on whether the run is nominal or adaptive, the configured script name, and whether the run is a single execution or part of a batch.

## Single-Run Outputs

Typical structure:

```text
results/
|-- nominal/
|   `-- <timestamp>_<traj>_<ctrl>_<potential>/
|       |-- command_window.txt
|       |-- metrics.txt
|       |-- summary_nominal.png
|       `-- sim_data.mat
`-- adaptive/
    `-- <timestamp>_<traj>_<ctrl>_<potential>/
        |-- command_window.txt
        |-- metrics.txt
        |-- summary_adaptive.png
        |-- stack_estimation.png
        `-- sim_data.mat
```

`sim_data.mat` is only present when the run is configured to save simulation data.

## Batch Outputs

Batch runs create a parent directory plus one folder per trajectory and per run label:

```text
results/adaptive/
`-- <timestamp>_<script-or-batch-name>/
    |-- adaptive_report.txt
    |-- command_window.txt
    |-- t01_circle/
    |   |-- baseline/
    |   |-- euclid-base-gain/
    |   `-- ...
    |-- t02_lissajous3d/
    `-- ...
```

Folder names vary with the configured `runNames`, script labels, and trajectory list.

## Common Artifacts

- `command_window.txt`: captured MATLAB console output
- `metrics.txt`: textual summary of tracking and estimation metrics
- `summary_*.png`: main summary figure for the run
- `stack_estimation.png`: additional adaptive estimation plot when enabled
- `sim_data.mat`: saved logs and configuration snapshot for later analysis
- `adaptive_report.txt`: aggregated batch summary

## Plot Modes

Typical run options use:

- `plotMode = 'summary'` for the main figure only
- `plotMode = 'all'` for additional figures
- `plotMode = 'none'` for headless or performance-focused execution

## Replotting Saved Runs

If `sim_data.mat` was saved, you can regenerate plots later:

```matlab
fth.io.ResultsManager.plotSavedRun( ...
    'results/adaptive/<saved-run-folder>', ...
    'summary', ...
    false);

fth.io.ResultsManager.plotSavedRun( ...
    'results/adaptive/<saved-run-folder>/sim_data.mat', ...
    'all', ...
    true);
```

## Practical Guidance

- Keep `saveSimData` off for large sweeps unless you need replotting or post-processing.
- Keep plotting hidden for CI runs.
- Use meaningful `runNames` in batch mode so result folders remain readable.
