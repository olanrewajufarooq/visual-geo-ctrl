"""Run one nominal visual geometric-control simulation."""

import argparse
import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(REPO_ROOT), str(REPO_ROOT / "src")]

from vgc.io.persistence import default_simulation_results_root, save_run
from vgc.sim.default_scenario import default_scenario
from vgc.sim.metrics import compute_metrics
from vgc.sim.run_scenario import run_scenario


def build_parser():
    parser = argparse.ArgumentParser(description="Run a nominal geometric-control simulation in PyBullet.")
    parser.add_argument("--replay-id", default="lemniscate_01_auto")
    parser.add_argument("--coriolis", type=str.lower, default="lc", choices=["lc", "rb"])
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--no-pacing", action="store_true")
    parser.add_argument("--ground", default="arena", choices=["arena", "grid", "plane"])
    parser.add_argument("--gates", default=None, choices=["lemniscate", "ratm", "auto", "none"])
    parser.add_argument("--drone-type", default="pybullet_drones", choices=["pybullet_drones", "hexacopter"])
    parser.add_argument("--cam-mode", default="chase", choices=["chase", "fpv", "overview", "free"])
    parser.add_argument("--ground-z", type=float, default=0.0)
    parser.add_argument("--osd", action="store_true")
    parser.add_argument("--output-dir")
    parser.add_argument("--timestamped-save", dest="inplace_save", action="store_false", default=True)
    return parser


def main():
    args = build_parser().parse_args()
    scenario = default_scenario(
        replay_id=args.replay_id, coriolis=args.coriolis, duration=args.duration,
        gui=args.gui, sim_speed=args.speed, enable_pacing=not args.no_pacing,
        ground_z=args.ground_z, ground_style=args.ground, gates_mode=args.gates,
        cam_mode=args.cam_mode, enable_osd=args.osd, drone_type=args.drone_type,
    )
    run, failure = run_scenario(scenario)
    results_dir = Path(args.output_dir) if args.output_dir else (
        default_simulation_results_root(str(REPO_ROOT), inplace_save=args.inplace_save)
        / f"nominal_{args.coriolis}"
    )
    os.makedirs(results_dir, exist_ok=True)
    metrics = compute_metrics(run) if failure is None else {"failed_time": failure["time"]}
    save_run(str(results_dir), run, metrics, scenario, failure=failure)
    print(f"Saved nominal {args.coriolis.upper()} run to {results_dir}")


if __name__ == "__main__":
    main()
