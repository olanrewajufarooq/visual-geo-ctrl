# AGENT.md

## Project overview

This is a paper-aligned Python/PyBullet simulator for a fully actuated UAV on `SE(3)`. PyBullet supplies floating-body multi-body forward dynamics, contact physics, and 3D visualization; project code implements paper equations, staged gain optimization, and automated experiment workflows.

## Development rules

- Production Python code belongs in `src/agc/`.
- `run/` files are CLI entry point scripts using `argparse`.
- Keep plant dynamics in `agc.plant`; PyBullet provides the physical plant backend.
- Pure numerical modules (`agc.math`, `agc.paper`, `agc.opt.bounds`) must remain importable without PyBullet.
- Keep controller equations in `agc.paper`; `C1(V)U` is `-ad(U)'*(I*V)`.
- Add a failing pytest unit test before production behavior changes.
- Preserve `trajectories/` and `assets/`; simulation defaults remain under `results/inplace/`. Gain optimization defaults to `results/optimization/best-gain/`; `--timestamped-save` selects a unique timestamped output directory. Optimization candidates and registry entries are isolated by controller mode and Coriolis form.
- No Python code reads or writes `.m` files; gain promotion updates `src/agc/config/optimized_gains.py` atomically.

## Commands

```powershell
conda activate agc
pytest tests/ -v
python run/run_sim.py --mode bregman --coriolis lc --duration 30
python run/run_batch_sim.py
python run/plot_trajectories.py --replay-id lemniscate_01_auto
```
