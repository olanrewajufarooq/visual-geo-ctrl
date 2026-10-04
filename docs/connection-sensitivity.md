# Connection sensitivity experiment

Run from the repository worktree with the `agc` environment:

```powershell
python run/run_paper_sim.py connection-sensitivity
```

This command generates every figure in `figures/04-nominal-validation` from the
same 4x study, including the LC/RB tracking, nominal reaching, and same-state
connection identity figures. The original 1x figures in that folder are superseded.
`all` runs the adaptive study followed by this study; `nominal-connection`,
`nominal-reaching`, and `connection-realizations` also generate this full nominal set.
The adaptive figures are unaffected by nominal-only commands. It always runs fresh; cache reuse and duration
overrides are rejected to preserve the specified source-time interval 10–30 s.

Both exact-inertia controllers use the original nominal gains and lemniscate.
Their common initial velocity perturbation is [1.2, -0.8, 0.4] rad/s and
[1.6, -0.8, 1.2] m/s; initial pose matches the reference at source time 10 s.
Only the Coriolis realization differs. This initialization is not a physical
payload-drop impulse. No optimization is run.

All figures use the full source-time interval 10--30 s. `T_obs` is reported
independently for LC and RB and denotes the first saved sample from which
`||s||_{Lambda_s} <= 1e-4` holds at every saved sample through source time
30 s. The complete interval is required; incomplete logs have no qualifying
`T_obs`. The finer 1 ms pair also covers the full run.

Outputs in `results/papers`:

- `figures/04-nominal-validation/connection_sensitivity_tracking.{pdf,png}`
- `figures/04-nominal-validation/connection_sensitivity_separation.{pdf,png}`
- `tables/connection_sensitivity_summary.csv`
- `metadata/connection_sensitivity_protocol.json`
- `metadata/connection_sensitivity_diagnostics.md`

Raw trajectories and scenario/source fingerprints are saved separately under
`results/paper-runs/connection-sensitivity`. The protocol JSON records all gains,
initial transverse errors, per-connection `T_obs` and persistence status, identity
residuals, command-replay checks, pair separations, and step-refinement measures.
The diagnostics file defines the metrics and their limitations. Never describe
the independent-trajectory wrench difference as the same-state identity.
