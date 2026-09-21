"""Generate static publication figures from saved simulation results."""

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "src"))

from agc.io.persistence import load_run
from agc.viz.paper_figures import export_run_figures


def main():
    results_base = REPO_ROOT / "results" / "pybullet"
    if not results_base.is_dir():
        print(f"No results directory found at {results_base}. Run run_theory_suite.py or run_batch.py first.")
        return

    variants = [d for d in results_base.iterdir() if d.is_dir() and (d / "run.npz").is_file()]
    if not variants:
        print("No completed runs found to generate figures from.")
        return

    for v_dir in variants:
        print(f"Generating figures for {v_dir.name}...")
        run = load_run(str(v_dir))
        export_run_figures(run, str(v_dir))
        print(f"  -> Wrote figures to {v_dir}")


if __name__ == "__main__":
    main()
