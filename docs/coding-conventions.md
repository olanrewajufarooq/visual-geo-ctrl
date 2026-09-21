# Coding Conventions

## Namespacing & Package Hierarchy

All Python modules reside under the `agc` package namespace:
- `agc.math`: Lie-group operations on $\mathrm{SE}(3)$ and $\mathrm{SO}(3)$, pseudo-inertia, and SPD validations.
- `agc.paper`: Paper equations, tracking error covectors, Coriolis factorizations (`c1`, `c2`), 6x10 regressor $Y$, and adaptation laws.
- `agc.plant`: PyBullet floating-base multi-body physics plant and link wrench interfaces.
- `agc.sim`: Multi-rate simulation execution (`run_scenario`), continuous trajectory replay, metrics, and scenario validation.
- `agc.opt`: Staged block-coordinate optimization, PSO, DE, Nelder-Mead simplex, and gain bounds.
- `agc.io`: Persistence utilities for `.npz` runs, `metadata.json`, and `manifest.json`.
- `agc.batch`: Batch simulation orchestration and process isolation.
- `agc.viz`: Headless publication figures and 3D PyBullet replay.

## Package Boundaries & PyBullet Isolation

- Pure numerical modules (`agc.math`, `agc.paper`, `agc.opt.bounds`) must remain importable without PyBullet.
- PyBullet dependencies must be localized to physics and 3D visualizer modules (`agc.plant`, `agc.viz.replay_3d`).

## Python Code Style

- Use Python 3.10+ typing annotations (`Dict[str, Any]`, `np.ndarray`, `Optional[float]`).
- All scripts under `run/` must expose clear CLI arguments via `argparse`.
- Avoid side-effects on module import.

## Automated Testing

- Add unit and regression tests under `tests/` using `pytest`.
- Test numerical algorithms against deterministic fixtures.
