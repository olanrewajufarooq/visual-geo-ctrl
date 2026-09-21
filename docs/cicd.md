# CI/CD

The repository includes GitHub Actions workflows under `.github/workflows/` for automated verification and release builds.

## 1. Continuous Integration (`ci.yml`)

### Triggers
- Pushes to the `main` branch
- Pull requests targeting `main`

### Pipeline Steps
1. Checks out repository and configures Python 3.12.
2. Installs dependencies (`pip install -e ".[dev]"`).
3. Executes the full automated test suite with pytest:
   ```bash
   pytest tests/ -v
   ```
4. Runs smoke simulation verification:
   ```bash
   python run/run_theory_suite.py --mode nominal --coriolis c1 --duration 0.1 --no-pacing
   python run/run_batch.py --duration 0.1 --serial
   ```

---

## 2. Release Results Workflow (`release-results.yml`)

### Triggers
- Tag pushes matching `v*` (e.g. `v1.0.0`).

### Pipeline Steps
1. Sets up Python 3.12 and installs dependencies.
2. Runs the full test suite.
3. Executes the full 6-variant comparative study batch:
   ```bash
   python run/run_batch.py --duration 30.0 --serial
   ```
4. Verifies output directory `results/timestamped/`.
5. Compresses the results into `results-<tag>.zip`.
6. Attaches the zip archive to the GitHub Release.
