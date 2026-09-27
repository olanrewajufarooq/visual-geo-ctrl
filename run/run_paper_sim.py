"""Run paper simulations and export figures; never run re-optimization."""
import argparse
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))
from agc.sim.publication_runner import run_publication


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", nargs="?", default="all", choices=["all", "adaptive-drop", "nominal-connection", "nominal-reaching", "connection-realizations", "connection-sensitivity"])
    parser.add_argument("--duration", type=float, default=30.)
    parser.add_argument("--output-dir", type=Path, default=REPO_ROOT/"results"/"papers")
    parser.add_argument("--raw-output-dir", type=Path, default=REPO_ROOT/"results"/"paper-runs")
    parser.add_argument("--reuse-cache", action="store_true", help="Reuse only raw runs matching the scenario and source fingerprints.")
    args = parser.parse_args()
    if args.duration <= 0: parser.error("duration must be positive")
    if args.command == "connection-sensitivity":
        if args.duration != 30. or args.reuse_cache:
            parser.error("connection-sensitivity uses fresh runs over source time 10–30 s")
        from agc.sim.connection_sensitivity import run_study
        run_study(args.output_dir, args.raw_output_dir)
        return
    run_publication(args.command, args.duration, args.output_dir, args.raw_output_dir, reuse_cache=args.reuse_cache)


if __name__ == "__main__":
    main()
