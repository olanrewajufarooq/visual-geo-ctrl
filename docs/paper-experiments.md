# Paper experiments

The paper experiment runner uses the `agc` Conda environment and writes
publication artifacts to `results/papers/`:

```text
conda activate agc
python run/run_paper_experiments.py
python run/run_paper_experiments.py adaptive-drop
python run/run_paper_experiments.py nominal-connection
python run/run_paper_experiments.py nominal-reaching
```

The adaptive release benchmark uses a 10 s release, a 5 cm position
threshold, a 5 degree geodesic attitude threshold, and a 1 s recovery
dwell. The commanded wrench is reported; the current plant does not expose
rotor allocation or actuator saturation.

Both adaptive controllers use the saved optimized Bregman/LC tracking
gains, held identical. Existing adaptation gains are reused provisionally.
No re-optimization is run; `agc.opt.paper_adaptation.adaptation_objective`
provides a future adaptation-only objective with the same initial state,
estimate, trajectory, release, duration, constraints and weights for both modes.
The gain summary explicitly records the tuning qualification. The old gains
were optimized before the PyBullet inertia/frame correction and should not be
described as optimal for the revised plant.

Outputs are grouped without an extra paper-name directory:

```text
results/papers/
  figures/01-adaptive-tracking/       # states, errors, wrench, 3-D
  figures/02-physical-consistency/    # mass and pseudo-inertia margin
  figures/04-nominal-validation/     # connection identity and reaching
  tables/baseline_comparison.csv
  gain_summary.json
  finite_time_reaching_summary.json
  connection_equivalence_summary.json
  metric_definitions.md
  diagnostic_report.md
  validation_status.json
  manifest.json
```

Each figure is saved as vector PDF and 400 dpi
PNG. Payload time histories use 0--30 s and mark 10 s. The reaching and
connection-equivalence plots share a focused transient window. Connection
equivalence retains its logarithmic residual panel and labels its time axis
"Time since initialization, t [s]". For reaching, a conservative bound outside the view
is annotated with its value and an arrow, never falsely drawn at the edge.
The numerical bound uses the specified eigenvalue estimate, not a tighter
replacement. Numerical reaching is threshold-and-dwell evidence only.

The Fixed-model controller is excluded from adaptive analysis and default runs.
The summary CSV now compares only Euclidean and Natural/Bregman adaptation.
Velocity
reference curves are transported into each actual body frame using the full
adjoint; these references need not coincide between controllers. Angle errors
in the CSV are intrinsic and in degrees; RPY figures are only visualization.

Raw logs and metadata are stored under ignored `results/paper-runs/`. Old flat
debug figures and deterministic repeatability tables are superseded; never cite
them as validation. Failed runs retain failure metadata and partial-data status;
failed reaching figures use a `_FAILED` filename. Read `diagnostic_report.md`
and `validation_status.json` before choosing manuscript figures.

Simulation-setup artwork is owned by a separate workflow and is not generated,
overwritten, or included by this pipeline.
