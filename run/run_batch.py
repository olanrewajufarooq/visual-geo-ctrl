"""Run all six paper comparison variants with PyBullet plant."""

import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "src"))

from agc.sim.default_scenario import default_scenario
from agc.batch.run_batch import run_batch
from agc.io.persistence import save_run
from agc.viz.paper_figures import export_run_figures


def main():
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
            replay_id="lemniscate_01_auto",
            mode=mode,
            coriolis=coriolis,
            duration=30.0,
            gain_source="optimized",
            gui=False,
        )
        for mode, coriolis in variants
    ]

    print("=" * 65)
    print("Running AGC 6-Variant Comparison Batch in PyBullet")
    print("=" * 65)

    batch = run_batch(scenarios)

    print("\n" + "=" * 65)
    print("Batch Results Summary:")
    print("=" * 65)

    for i, (mode, coriolis) in enumerate(variants):
        name = f"{mode}_{coriolis}"
        run = batch["runs"][i]
        metrics = batch["metrics"][i]
        failure = batch["failures"][i]

        out_dir = REPO_ROOT / "results" / "pybullet" / name
        os.makedirs(out_dir, exist_ok=True)

        if failure is None and metrics is not None:
            print(f"{name:15s} | Pos RMSE: {metrics['positionRMSE']:.4f} m | Att RMSE: {metrics['attitudeRMSE']:.4f} rad | Final ||s||: {metrics['finalSlidingNorm']:.4f}")
            save_run(str(out_dir), run, metrics, scenarios[i])
            export_run_figures(run, str(out_dir))
        else:
            print(f"{name:15s} | FAILED at t = {failure['time']:.3f} s: {failure['message']}")


if __name__ == "__main__":
    main()
