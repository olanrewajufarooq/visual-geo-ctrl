"""Run all six paper comparison variants with PyBullet plant."""

import os
import sys
import argparse
from datetime import datetime
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "src"))

from agc.sim.default_scenario import default_scenario
from agc.batch.run_batch import run_batch
from agc.io.persistence import save_batch_suite, default_simulation_results_root
from agc.viz.paper_figures import export_run_figures, export_suite_comparison_figures


def main():
    parser = argparse.ArgumentParser(description="Run AGC 6-Variant Comparison Batch in PyBullet.")
    parser.add_argument("--duration", type=float, default=30.0, help="Simulation duration (seconds)")
    parser.add_argument("--replay-id", type=str, default="lemniscate_01_auto", help="Trajectory artifact ID")
    parser.add_argument("--serial", action="store_true", help="Run scenarios sequentially (single process)")
    parser.add_argument("--parallel", action="store_true", help="Run scenarios in parallel using worker processes")
    parser.add_argument("--output-dir", type=str, default=None, help="Custom output directory for suite results")
    parser.add_argument("--no-figures", action="store_true", help="Skip figure generation")
    parser.add_argument("--show-figures", action="store_true", help="Show generated Matplotlib figures")
    parser.add_argument(
        "--inplace-save",
        dest="inplace_save",
        action="store_true",
        default=True,
        help="Save under results/inplace (default)",
    )
    parser.add_argument(
        "--timestamped-save",
        dest="inplace_save",
        action="store_false",
        help="Save under results/timestamped/<timestamp>",
    )
    args = parser.parse_args()

    # Determine parallelism
    is_parallel = True
    if args.serial:
        is_parallel = False
    elif args.parallel:
        is_parallel = True

    variants = [
        ("nominal", "c1"),
        ("nominal", "c2"),
        ("euclidean", "c1"),
        ("euclidean", "c2"),
        ("bregman", "c1"),
        ("bregman", "c2"),
    ]

    scenarios = [
        default_scenario(
            replay_id=args.replay_id,
            mode=mode,
            coriolis=coriolis,
            duration=args.duration,
            gain_source="optimized",
            gui=False,
        )
        for mode, coriolis in variants
    ]

    print("=" * 65)
    print(f"Running AGC 6-Variant Comparison Batch in PyBullet ({'PARALLEL' if is_parallel else 'SERIAL'})")
    print(f"Trajectory: {args.replay_id} | Duration: {args.duration} s")
    print("=" * 65)

    batch = run_batch(scenarios, parallel=is_parallel)

    # Determine suite directory
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    if args.output_dir is not None:
        suite_dir = Path(args.output_dir)
    else:
        suite_dir = default_simulation_results_root(
            str(REPO_ROOT), inplace_save=args.inplace_save, timestamp=stamp
        )

    # Save suite manifest and per-variant data
    save_batch_suite(str(suite_dir), scenarios, batch)

    print("\n" + "=" * 65)
    print(f"Batch Results Summary (Saved to {suite_dir}):")
    print("=" * 65)

    for i, (mode, coriolis) in enumerate(variants):
        name = f"{mode}_{coriolis}"
        run = batch["runs"][i]
        metrics = batch["metrics"][i]
        failure = batch["failures"][i]

        if failure is None and metrics is not None:
            print(f"{name:15s} | Pos RMSE: {metrics['positionRMSE']:.4f} m | Att RMSE: {metrics['attitudeRMSE']:.4f} rad | Final ||s||: {metrics['finalSlidingNorm']:.4f}")
            if not args.no_figures:
                export_run_figures(run, str(suite_dir / name), visible=args.show_figures)
        else:
            print(f"{name:15s} | FAILED at t = {failure['time']:.3f} s: {failure['message']}")

    if not args.no_figures:
        print("\nExporting suite-level comparison figures...")
        export_suite_comparison_figures(str(suite_dir), visible=args.show_figures)


if __name__ == "__main__":
    main()
