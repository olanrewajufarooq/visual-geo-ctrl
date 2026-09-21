# Customization

The codebase is modular, making it straightforward to add new trajectory profiles, custom gain schedules, or alternative adaptation laws.

## Adding a New Adaptation Mode

1. Implement the discrete update law in `src/agc/paper/adaptation.py`:
   ```python
   def custom_adaptation_step(hat_pi, Y, s, gamma, dt):
       ...
       return hat_pi_next
   ```
2. Integrate the mode in `src/agc/sim/run_scenario.py` within the adaptation tick branch.
3. Update parameter bounds and gain encoding in `src/agc/opt/bounds.py` and `src/agc/opt/encoding.py`.
4. Add unit tests in `tests/test_paper_core.py`.

## Adding a Custom Gain Schedule

In `src/agc/opt/bounds.py`, define custom block-coordinate optimization stages:
```python
def custom_optimization_stages(mode: str) -> List[str]:
    return ["tracking", "damping", "adaptive", "all"]
```

## Adding Benchmark Trajectories

1. Place processed trajectory `.mat` files in `trajectories/processed/`.
2. Ensure the file contains monotonic `t`, positions `p`, velocities `v`, accelerations `a`, and orientations `R`.
3. Verify using:
   ```powershell
   python run/plot_trajectories.py --replay-id <new_trajectory_id>
   ```
