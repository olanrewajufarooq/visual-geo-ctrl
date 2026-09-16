# AGENT.md

## Project overview

This is a paper-aligned MATLAB reference simulator for a fully actuated UAV on `SE(3)`. Robotics System Toolbox supplies floating-body forward dynamics and plotting; project code implements paper equations and experiment workflows.

## Development rules

- Production MATLAB code belongs in `src/+agc/`.
- `run/` files are scripts with editable settings, never functions.
- Keep plant dynamics in `agc.plant`; do not add a hand-coded rigid-body plant.
- Keep controller equations in `agc.paper`; `C1(V)U` is `-ad(U)'*(I*V)`.
- Add a failing MATLAB unit test before production behavior changes.
- Preserve `trajectories/`, `assets/`, and historical `results/`; new output belongs in `results/v2/`.

## Commands

```matlab
startup
runtests('tests')
run_theory_suite
run_batch
```
