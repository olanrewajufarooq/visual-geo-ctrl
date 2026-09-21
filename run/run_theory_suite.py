"""Run paper theory tracking scenarios with PyBullet simulation."""

import os
import sys
import argparse
from pathlib import Path

# Add repo and src to path
REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "src"))

from agc.sim.default_scenario import default_scenario
from agc.sim.run_scenario import run_scenario
from agc.sim.metrics import compute_metrics
from agc.io.persistence import save_run
from agc.viz.paper_figures import export_run_figures


def main():
    parser = argparse.ArgumentParser(description="Run AGC UAV theory tracking simulation in PyBullet.")
    parser.add_argument("--replay-id", type=str, default="lemniscate_01_auto", help="Trajectory artifact ID")
    parser.add_argument("--mode", type=str, default="bregman", choices=["nominal", "euclidean", "bregman"], help="Controller mode")
    parser.add_argument("--coriolis", type=str, default="c1", choices=["c1", "c2"], help="Coriolis factorization")
    parser.add_argument("--duration", type=float, default=30.0, help="Simulation duration (seconds)")
    parser.add_argument("--gui", action="store_true", help="Launch interactive 3D PyBullet GUI")
    parser.add_argument("--speed", type=float, default=1.0, help="GUI playback speed multiplier (e.g. 1.0 for real-time, 2.0 for 2x)")
    parser.add_argument("--no-pacing", action="store_true", help="Disable wall-clock real-time pacing")
    parser.add_argument("--save-figures", action="store_true", default=True, help="Export paper figures")
    parser.add_argument("--output-dir", type=str, default=None, help="Custom output directory")
    args = parser.parse_args()

    print("=" * 60)
    print(f"Running AGC Simulation: Mode = {args.mode}, Coriolis = {args.coriolis}")
    print(f"Trajectory = {args.replay_id}, Duration = {args.duration} s, GUI = {args.gui}, Speed = {args.speed}x")
    print("=" * 60)

    scenario = default_scenario(
        replay_id=args.replay_id,
        mode=args.mode,
        coriolis=args.coriolis,
        duration=args.duration,
        gain_source="optimized",
        gui=args.gui,
        sim_speed=args.speed,
        enable_pacing=not args.no_pacing,
    )

    run, failure = run_scenario(scenario)

    if args.output_dir is not None:
        results_dir = Path(args.output_dir)
    else:
        results_dir = REPO_ROOT / "results" / "pybullet" / f"{args.mode}_{args.coriolis}"
    os.makedirs(results_dir, exist_ok=True)

    if failure is None:
        metrics = compute_metrics(run)
        print("\nSimulation Succeeded!")
        print(f"Position RMSE:           {metrics['positionRMSE']:.4f} m")
        print(f"Attitude RMSE:           {metrics['attitudeRMSE']:.4f} rad")
        print(f"Linear Velocity RMSE:    {metrics['linearVelocityRMSE']:.4f} m/s")
        print(f"Angular Velocity RMSE:   {metrics['angularVelocityRMSE']:.4f} rad/s")
        print(f"Max Position Error:      {metrics['maxPositionError']:.4f} m")
        print(f"Wrench RMS:              {metrics['wrenchRMS']:.4f} N/Nm")
        print(f"Mass Estimation RMSE:    {metrics['massEstimationRMSE']:.4f}")
        print(f"CoM Estimation RMSE:     {metrics['centerOfMassEstimationRMSE']:.4f}")
        print(f"Final Sliding Norm:      {metrics['finalSlidingNorm']:.4f}")
        print(f"Max Potential (Psi):     {metrics['maxPsi']:.4f}")
        if args.mode == "bregman":
            print(f"Min Pseudo-Eigenvalue:   {metrics['minimumPseudoEigenvalue']:.6f} (>0 verifies SPD)")

        save_run(str(results_dir), run, metrics, scenario)
        print(f"Saved run data to: {results_dir}")

        if args.save_figures:
            export_run_figures(run, str(results_dir))
            print(f"Saved publication figures to: {results_dir}")
    else:
        print(f"\nSimulation Failed at t = {failure['time']:.3f} s: {failure['message']}")
        metrics = {"failed_time": failure["time"]}
        save_run(str(results_dir), run, metrics, scenario)


if __name__ == "__main__":
    main()
