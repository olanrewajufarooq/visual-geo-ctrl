# Troubleshooting

## Python Environment & Import Issues

### Symptoms:
- `ModuleNotFoundError: No module named 'agc'`
- `ModuleNotFoundError: No module named 'pybullet'` or `numpy`

### Solution:
Verify that the `agc` Conda environment is active:
```powershell
conda activate agc
```
If developing in editable mode, reinstall the package:
```powershell
pip install -e ".[dev]"
```
Test module imports:
```powershell
python -c "import agc.math, agc.paper, agc.plant; print('Imports OK')"
```

---

## PyBullet GUI & Visualization Issues

### Symptoms:
- PyBullet window crashes or fails to open with `--gui`.
- OpenGL/driver warning on headless or virtual machines.

### Solutions:
1. Run in headless mode:
   ```powershell
   python run/run_sim.py --mode bregman --coriolis lc
   ```
2. Update OpenGL / GPU display drivers. PyBullet utilizes hardware OpenGL acceleration for the 3D window when `--gui` is specified.
3. For remote machines or Docker containers, use `pybullet.DIRECT` (default in batch and test scripts).

---

## Numerical Stability & Divergence

### Symptoms:
- Vehicle tumbles or divergence warning.
- Non-finite tracking errors or parameter estimates.

### Solutions:
1. Ensure the simulation rates maintain integer step ratios (`dtPlant = 0.002`, `dtControl = 0.02`, `dtAdaptation = 0.01`).
2. Verify positive definiteness of metric matrix $\Lambda$ and tracking gains $K_R, K_\xi$.
3. When using Euclidean adaptation, ensure learning rate $\gamma_E$ does not cause pseudo-inertia estimate $\hat{\mathcal{J}}$ to lose positive definiteness. Bregman adaptation (`--mode bregman`) is mathematically guaranteed to preserve SPD estimates for all finite time.

---

## Batch Process Isolation & Multi-Processing

### Symptoms:
- Parallel batch runs (`run_batch_sim.py`) stall or encounter pickling errors.

### Solutions:
1. Run in serial mode for debugging:
   ```powershell
   python run/run_batch_sim.py --serial --duration 5.0
   ```
2. On Windows, Python uses `spawn` for multiprocessing. All scenario arguments and callable wrappers must be top-level picklable objects.
