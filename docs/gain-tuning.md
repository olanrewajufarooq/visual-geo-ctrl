# Gain tuning

`run/optimize_gains.py` tunes the 15 nominal gain values against the same closed-loop simulation used by `run/run_sim.py`. LC and RB are optimized independently; a candidate is scored using position, attitude, velocity, force, and torque tracking terms.

Run the tuner after installing the project dependencies:

```bash
python run/optimize_gains.py --coriolis all --duration 10 --maxiter 20 --write
```

The committed manual registry contains the nominal LC/RB values promoted from the predecessor project’s verified tuning run. The LC predecessor report recorded 0.000481 m pre-event position RMSE and 0.111 degrees attitude RMSE for the known-inertia controller. The tuner’s JSON report is written to `results/optimization/nominal_gains.json`.
