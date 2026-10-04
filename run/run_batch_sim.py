"""Run the nominal LC/RB comparison batch."""

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(ROOT), str(ROOT / "src")]

from vgc.batch.run_batch import run_batch
from vgc.io.persistence import default_simulation_results_root, save_batch_suite
from vgc.sim.default_scenario import default_scenario


def main():
    parser = argparse.ArgumentParser(description="Run nominal LC/RB simulations.")
    parser.add_argument("--replay-id", default="lemniscate_01_auto")
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--timestamped-save", action="store_true")
    args = parser.parse_args()
    scenarios = [default_scenario(replay_id=args.replay_id, coriolis=form, duration=args.duration)
                 for form in ("lc", "rb")]
    batch = run_batch(scenarios, max_workers=args.workers)
    root = default_simulation_results_root(str(ROOT), inplace_save=not args.timestamped_save)
    save_batch_suite(str(root / "nominal_comparison"), scenarios, batch)
    print(f"Saved nominal batch to {root / 'nominal_comparison'}")


if __name__ == "__main__":
    main()
