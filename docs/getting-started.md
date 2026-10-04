# Getting started

Install the package and development dependencies:

```bash
pip install -e .[dev]
```

Run a headless simulation:

```bash
python run/run_sim.py --duration 30 --coriolis lc
```

Add `--gui` for the PyBullet visualizer. Use `--gates none` to disable gate geometry and `--no-figures` to skip figure export.
