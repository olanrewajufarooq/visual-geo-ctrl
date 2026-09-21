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
- Preserve `trajectories/` and `assets/`; normal results are saved to `results/timestamped/<timestamp>/`, or `results/inplace/` with `--inplace-save`.
- No Python code reads or writes `.m` files; gain promotion updates `config/optimized_gains.py`.

## Commands

```powershell
conda activate agc
pytest tests/ -v
python run/run_theory_suite.py --mode bregman --coriolis c1 --duration 30
python run/run_batch.py
python run/plot_trajectories.py --replay-id lemniscate_01_auto
```
