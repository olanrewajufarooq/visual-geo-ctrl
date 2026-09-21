"""Generate static publication figures from saved simulation results or batch suites."""

import sys
import argparse
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "src"))

from agc.io.persistence import load_run, resolve_result_suite
from agc.viz.paper_figures import export_run_figures, export_suite_comparison_figures


def main():
    parser = argparse.ArgumentParser(description="Generate publication figures from simulation results or suite.")
    parser.add_argument("suite_dir", nargs="?", default=None, help="Path to result run or suite directory")
    args = parser.parse_args()

    try:
        suite_path = resolve_result_suite(args.suite_dir)
    except FileNotFoundError as exc:
        print(f"Error: {exc}")
        return

    print(f"Generating figures for suite: {suite_path}")

    # Generate suite-level comparisons if multiple variants exist
    comp_figs = export_suite_comparison_figures(str(suite_path))
    for f in comp_figs:
        print(f"  -> Generated comparison figure: {f}")

    # Generate per-variant figures
    variants = [d for d in suite_path.iterdir() if d.is_dir() and (d / "run.npz").is_file()]
    for v_dir in variants:
        print(f"Generating run figures for {v_dir.name}...")
        try:
            run = load_run(str(v_dir))
            figs = export_run_figures(run, str(v_dir))
            for f in figs:
                print(f"    -> Wrote {Path(f).name}")
        except Exception as exc:
            print(f"    -> Skipped {v_dir.name}: {exc}")


if __name__ == "__main__":
    main()
