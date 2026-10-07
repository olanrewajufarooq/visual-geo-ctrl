"""Compare manual and optimized nominal gains in short PyBullet runs."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(ROOT), str(ROOT / "src")]

from vgc.config.manual_gains import manual_gains
from vgc.config.optimized_gains import optimized_gains
from vgc.sim.default_scenario import default_scenario
from vgc.sim.metrics import compute_metrics
from vgc.sim.run_scenario import run_scenario


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--output", type=Path, default=ROOT / "results" / "optimization" / "nominal_gain_validation.json")
    args = parser.parse_args()

    report = {}
    for coriolis in ("lc", "rb"):
        report[coriolis] = {}
        for label, gains in (("manual", manual_gains("nominal", coriolis)),
                             ("optimized", optimized_gains("nominal", coriolis))):
            scenario = default_scenario(
                replay_id="lemniscate_01_auto",
                coriolis=coriolis,
                duration=args.duration,
                gui=False,
                enable_pacing=False,
                gain_override=gains,
            )
            run, failure = run_scenario(scenario)
            report[coriolis][label] = {
                "failure": failure,
                "metrics": compute_metrics(run) if failure is None else {},
            }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
