# Project structure

```text
visual-geo-ctrl/
  src/vgc/       Python package
  run/           CLI entry points
  trajectories/  Reference data and processing scripts
  assets/        Vehicle and gate assets
  tests/         Regression tests
  docs/          Project documentation
```

The package is organized by responsibility: `sim` runs scenarios, `paper` contains the nominal geometric controller, `plant` wraps PyBullet, `math` contains SE(3)/inertia utilities, `viz` owns rendering and telemetry, and `io` persists results.
