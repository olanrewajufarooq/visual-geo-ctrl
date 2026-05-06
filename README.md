# Adaptive Geo Ctrl

Simulink-first fixed-tilt hexacopter simulation for MATLAB R2024b.

Milestone 1 makes `Simulink + Simscape Multibody` the active runtime. MATLAB now handles configuration, trajectory generation, experiment orchestration, and post-processing. The old MATLAB-loop plant/runner remain as legacy reference code and are not the primary path.

## Current Scope

- Native Simscape Multibody plant generated as `vt_fixed_tilt_hex.slx`
- Nominal wrench controller in the active path
- Fixed-tilt fully actuated hexacopter allocation with pseudoinverse `B`-matrix allocation
- URDF used only as a parameter source
- Existing tracking metrics and plots reused through log conversion

Not in milestone 1:

- Adaptive runtime in the active path
- Variable tilt actuation
- Live custom visualization beyond Simulink/plot outputs

## Active Runtime

The active flow is:

```text
vt.config.Config
  -> vt.model.buildModelContext
  -> vt.model.buildFixedTiltModel
  -> vt.sim.ExperimentRunner.run
  -> sim()
  -> vt.sim.simOutputToLogs
  -> vt.metrics.TrackingMetrics / vt.plot.Plotter
```

Key active classes and functions:

- `vt.config.Config`: runtime, solver, tilt, allocator, controller, and trajectory configuration
- `vt.sim.ExperimentRunner`: Simulink-first runner
- `vt.model.extractFixedTiltParams`: URDF parameter extraction
- `vt.model.buildFixedTiltModel`: Simscape model generator
- `vt.ctrl.computeNominalWrench`: stateless nominal controller math
- `vt.alloc.buildEffectivenessMatrix`: fixed-tilt wrench effectiveness matrix
- `vt.alloc.allocateWrench`: pseudoinverse allocator with saturation
- `vt.sim.simOutputToLogs`: `SimulationOutput` to legacy-style `logs` conversion

## Quick Start

```matlab
startup
run_nominal_demo
```

Or configure directly:

```matlab
startup

cfg = vt.config.Config();
cfg.setSimulinkModel('vt_fixed_tilt_hex');
cfg.setControlParams(0.01);
cfg.setSimParams(0.01, 20);
cfg.setTrajectory('infinity3d', 1.2);
cfg.setTrajectoryMethod('precomputed');
cfg.setController('Feedforward');
cfg.setPotentialType('liealgebra');
cfg.setAdaptation('none');
cfg.setKpGains([5.5; 5.5; 5.5; 5.5; 5.5; 5.5]);
cfg.setKdGains([2.05; 2.05; 2.05; 2.05; 2.05; 2.05]);
cfg.setFixedTiltAngles(deg2rad([20; -20; 20; -20; 20; -20]));
cfg.setAllocatorMethod('pseudoinverse');
cfg.done();

runner = vt.sim.ExperimentRunner(cfg);
[logs, metrics, simOut] = runner.run('summary', false, true);
```

## Configuration Notes

Important active settings:

- `cfg.setSimulinkModel(name)`: primary model name
- `cfg.setFixedTiltAngles(x)`: scalar, `1x6`, or `6x1` tilt input in radians
- `cfg.setAllocatorMethod('pseudoinverse')`: only supported active allocator in milestone 1
- `cfg.setControlParams(dt)`: discrete controller/allocator sample time
- `cfg.setSimParams(dt, duration)`: analysis sampling step and duration

Default solver behavior:

- Simscape plant runs with a variable-step solver
- Controller/allocation logic runs at the configured discrete control step
- outputs are resampled onto `cfg.sim.analysis_dt` for deterministic analysis

## Outputs

The runner writes standard repo outputs under `results/nominal/...`:

- `metrics.txt`
- `sim_data.mat` when enabled
- summary/all plots when requested

The converted `logs` struct keeps the downstream analysis contract:

- `logs.t`
- `logs.actual`
- `logs.des`
- `logs.cmd`
- `logs.timing`

Additional command fields are included for the Simulink path:

- `logs.cmd.commandedWrenchF`
- `logs.cmd.commandedWrenchT`
- `logs.cmd.achievedWrenchF`
- `logs.cmd.achievedWrenchT`
- `logs.cmd.rotorThrustCmd`
- `logs.cmd.rotorThrustRealized`

## Legacy Runtime

The older MATLAB-loop runtime is retained only for reference.

- `vt.sim.SimRunner`
- `vt.plant.HexacopterPlant`
- adaptive MATLAB-loop scripts

See [legacy/README.md](legacy/README.md).

## Tests

Validated in MATLAB R2024b with:

```matlab
runtests('tests/TestFixedTiltRuntime.m')
runtests('tests/TestConfig.m')
runtests('tests/TestWrenchController.m')
```

## Limitations

- The active runtime is nominal-only.
- Adaptive scripts are intentionally not active in milestone 1.
- The generated model name `vt_fixed_tilt_hex` may produce a MATLAB warning while being rebuilt because the model file already exists on disk. This does not block simulation.
