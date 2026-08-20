# Customization

This framework is structured so new controllers, trajectories, and visualization behaviors can be added without rewriting the simulation runner.

## Adding a New Controller

1. Add the controller label to the code path that configures controller behavior in `src/+fth/+sim/Config.m` or the struct-based script helpers that feed it.
2. Implement the new wrench-generation branch in `src/+fth/+ctrl/ControllerWrench.m`.
3. If construction logic changes, update `src/+fth/+ctrl/ControllerFactory.m`.
4. Add or update tests in `tests/` covering the new behavior.
5. Validate through a focused run script or a small local scenario.

## Adding a New Adaptation Strategy

1. Create the class under `src/+fth/+ctrl/+adapt/`.
2. Register it in `src/+fth/+ctrl/+adapt/AdaptationFactory.m`.
3. Ensure gain-shape expectations are reflected in configuration handling.
4. Add math and interface tests alongside the existing adaptation tests.

## Adding a New Trajectory

1. Add the implementation under `src/+fth/+traj/`, usually by following `TrajectoryBase`.
2. Register the new name in `src/+fth/+traj/TrajectoryFactory.m`.
3. Update the supported-name handling in `src/+fth/+sim/Config.m`.
4. Add a lightweight validation path through `run/plot_trajectories.m` or a dedicated test.

## Adjusting Visualization

The config API exposes the main visualization controls:

```matlab
cfg.enableLiveView(true);
cfg.setLiveSummary(true);
cfg.setLiveUpdateRate(200);
cfg.setLiveUrdfEmbedding(true);
cfg.setPlotLayout('row-major');
```

Lower-level defaults live under `cfg.viz`, including `dynamicAxis`, `axisPadding`, and `initialAxis`.

## Custom Run Scripts

The simplest extension path is often to add a new file under `run/` that:

1. Calls `startup`
2. Builds `fth.sim.Config`
3. Applies scenario-specific structs or fluent setters
4. Finalizes with `cfg.done()`
5. Executes `SimRunner`

This keeps repeatable experiments out of ad hoc command-window history.
