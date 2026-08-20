# Configuration

`fth.sim.Config` is the main configuration surface for simulations. It stores vehicle, simulation, trajectory, controller, visualization, payload, and actuation settings, and it supports both fluent setters and grouped `use...Options(...)` helpers.

## Basic Example

```matlab
startup

cfg = fth.sim.Config();
cfg.setTrajectory('lissajous3d');
cfg.setController('Feedforward', 'inertia-gain');
cfg.setAdaptation('euclidean');
cfg.setSimParams(0.005, 30);
cfg.setControlParams(0.01);
cfg.setAdaptationParams(0.005);
cfg.done();
```

## Adaptive Payload-Drop Example

```matlab
startup

cfg = fth.sim.Config();
duration = 30;

cfg.setTrajectory('lissajous3d');
cfg.setController('Feedforward', 'inertia-gain');
cfg.setAdaptation('euclidean');
cfg.setSimParams(0.005, duration);
cfg.setControlParams(0.01);
cfg.setAdaptationParams(0.005);
cfg.setAdaptiveGains(1e-2 * [36, 12, 12, 12, 8, 8, 12, 0.4, 0.4, 0.4]);
cfg.setPayloadScenario(1.5, [0.115; 0.05; -0.25], 2 * duration / 3);
cfg.setPayloadDims([0.20, 0.20, 0.10]);
cfg.setParamInit('mid-vehicle-payload');
cfg.done();
```

## Core Configuration Areas

### Trajectory

```matlab
cfg.setTrajectory('hover');
cfg.setTrajectory('circle', true);
cfg.setTrajectory({'circle', 'lissajous3d', 'helix3d'}, [true false false]);
```

Built-in names include `hover`, `circle`, `infinity`, `lissajous3d`, `helix3d`, `poly3d`, and `takeoffland`.

### Controller and Adaptation

```matlab
cfg.setController('PD', 'log');
cfg.setController('Feedforward', 'inertia-gain');
cfg.setController('FeedLin', 'body-gain');

cfg.setAdaptation('none');
cfg.setAdaptation('euclidean');
cfg.setAdaptation('bregman');
```

### Timing

```matlab
cfg.setSimParams(0.005, 30);
cfg.setControlParams(0.01);
cfg.setAdaptationParams(0.005);
```

`cfg.done()` resolves final timing relationships and default gains.

### Gains

```matlab
cfg.setKpGains([5.5 5.5 5.5 5.5 5.5 5.5]);
cfg.setKdGains([2.05 2.05 2.05 2.05 2.05 2.05]);
cfg.setAdaptiveGains(1e-2 * [36 12 12 12 8 8 12 0.4 0.4 0.4]);
cfg.setLambda(1e-3 * [5 5 5 50 50 50]);
```

`setAdaptiveGains(...)` accepts scalar, vector, or batched values depending on adaptation mode.

### Payload and Parameter Initialization

```matlab
cfg.setPayloadScenario(1.5, [0.115; 0.05; -0.25], 20);
cfg.setPayloadDims([0.20, 0.20, 0.10]);
cfg.setParamInit('vehicle');
cfg.setParamInit('mid-vehicle-payload');
cfg.setParamInit('random');
```

### Visualization

```matlab
cfg.enableLiveView(true);
cfg.setLiveSummary(true);
cfg.setLiveUpdateRate(100);
cfg.setLiveUrdfEmbedding(false);
cfg.setPlotLayout('column-major');
```

## Struct-Based Helper API

The run scripts use grouped helpers such as:

- `cfg.useSimOptions(simOpts)`
- `cfg.useTrajectoryOptions(trajOpts)`
- `cfg.useControllerOptions(ctrlOpts)`
- `cfg.useAdaptationOptions(adaptOpts)`
- `cfg.usePayloadOptions(payloadOpts)`
- `cfg.useVizOptions(vizOpts)`

These are useful when a script wants to assemble scenario-specific structs before applying them to the config.

## Full Reference Summary

| Area | Representative methods |
| --- | --- |
| Trajectory | `setTrajectory`, `useTrajectoryOptions` |
| Controller | `setController`, `setKpGains`, `setKdGains`, `setLambda`, `useControllerOptions` |
| Adaptation | `setAdaptation`, `setAdaptiveGains`, `useAdaptationOptions` |
| Timing | `setSimParams`, `setControlParams`, `setAdaptationParams`, `useSimOptions` |
| Payload | `setPayloadScenario`, `setPayloadDims`, `setParamInit`, `usePayloadOptions` |
| Visualization | `enableLiveView`, `setLiveSummary`, `setLiveUpdateRate`, `setLiveUrdfEmbedding`, `setPlotLayout`, `useVizOptions` |

## Finalization Pattern

The usual execution order is:

```matlab
cfg = fth.sim.Config();
% configure fields here
cfg.done();

sim = fth.sim.SimRunner(cfg);
sim.setup();
sim.run(runOpts);
```
