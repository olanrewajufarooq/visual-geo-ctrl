# Paper experiments

The paper experiment runner uses the `agc` Conda environment and writes
publication artifacts to `results/papers/`:

```text
conda activate agc
python run/run_paper_sim.py
python run/run_paper_sim.py adaptive-drop
python run/run_paper_sim.py nominal-connection
python run/run_paper_sim.py nominal-reaching
python run/run_paper_sim.py connection-realizations
python run/run_paper_sim.py connection-sensitivity
python run/run_paper_sim.py physical-consistency-monte-carlo
```

The adaptive release benchmark uses a 10 s release, a 5 cm position
threshold, a 5 degree geodesic attitude threshold, and a 1 s recovery
dwell. The commanded wrench is reported; the current plant does not expose
rotor allocation or actuator saturation.

The payload-release comparison runs Known-inertia, Euclidean, and
Natural/Bregman controllers with both LC and RB realizations. Each run uses
its corresponding committed gain set: `nominal_lc`, `nominal_rb`,
`euclidean_lc`, `euclidean_rb`, `bregman_lc`, or `bregman_rb`. The
Known-inertia controller receives the active plant inertia across payload
release; it is not the separate nominal-validation controller. Nominal
connection and reaching studies use a payload-disabled scenario with their
own gains.

The runner reads configured gains and does not run PSO. Estimator adaptation
rates are used as saved; the run metadata does not claim they were fairly
retuned between methods. Gain provenance is recorded in
`metadata/gain_summary.json`, which distinguishes the payload-release and
nominal-validation protocols.

Outputs are grouped without an extra paper-name directory:

```text
results/papers/
  figures/01-adaptive-tracking/       # states, errors, wrench, 3-D
  figures/02-physical-consistency/    # mass and pseudo-inertia margin
  figures/04-nominal-validation/     # all nominal figures use the 4x sensitivity pair
  tables/adaptive_performance_summary.csv
  tables/controller_gain_summary.csv
  tables/physical_consistency_summary.csv
  tables/connection_realization_summary.csv
  tables/nominal_reaching_summary.csv
  metadata/                          # protocols, summaries, definitions, diagnostics, provenance
  figures/03-monte-carlo-verification/ # paired Monte Carlo figures
  metadata/physical_consistency_monte_carlo_*.json
  tables/physical_consistency_monte_carlo_*.csv
```

Each figure is saved as vector PDF and 400 dpi PNG. Payload histories use
0--30 s; connection histories use the full source interval 10--30 s. The
reaching threshold is 1e-4 and must hold at every saved sample through 30 s;
LC and RB reaching times are independent. The nominal bound includes both
the linear and fractional dissipation terms. Figures have no titles, use
full-intensity RGB primaries, and place legends above the axes.

The adaptive comparison includes the Known-inertia controller, which uses the
true loaded inertia before release and the true bare-vehicle inertia after
release, plus Euclidean and Natural/Bregman adaptation. Each mode/factorization
run uses its own committed tracking and sliding gains. The comparison does not
establish that `gamma_E` and `gamma_B` were tuned under an identical protocol,
so it makes no fair estimator-gain tuning claim. Velocity reference curves
are transported into each actual body frame using the full
adjoint; these references need not coincide between controllers. Angle errors
in the CSV are intrinsic and in degrees; RPY figures are only visualization.

Raw logs and metadata are stored under ignored `results/paper-runs/`. Old flat
debug figures and deterministic repeatability tables are superseded; never cite
them as validation. Failed runs retain failure metadata and partial-data status;
failed reaching figures use a `_FAILED` filename. Read
`metadata/diagnostic_report.md` and the report in `metadata/manifest.json`
before choosing manuscript figures.

Simulation-setup artwork is owned by a separate workflow and is not generated,
overwritten, or included by this pipeline.
