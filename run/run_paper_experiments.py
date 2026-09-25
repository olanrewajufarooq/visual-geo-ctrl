"""Run all publication experiments by default; never run re-optimization."""
import argparse
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))
from agc.sim.publication_runner import run_publication


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", nargs="?", default="all", choices=["all", "adaptive-drop", "nominal-connection", "nominal-reaching"])
    parser.add_argument("--duration", type=float, default=30.)
    parser.add_argument("--output-dir", type=Path, default=REPO_ROOT/"results"/"papers")
    parser.add_argument("--raw-output-dir", type=Path, default=REPO_ROOT/"results"/"paper-runs")
    args = parser.parse_args()
    if args.duration <= 0: parser.error("duration must be positive")
    run_publication(args.command, args.duration, args.output_dir, args.raw_output_dir)


if __name__ == "__main__":
    main()
