"""Run single UAV tracking scenario with PyBullet simulation."""

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
from agc.io.persistence import save_run, default_simulation_results_root
from agc.viz.paper_figures import export_run_figures


def main():
    parser = argparse.ArgumentParser(description="Run single AGC UAV tracking simulation in PyBullet.")
    parser.add_argument("--replay-id", type=str, default="lemniscate_01_auto", help="Trajectory artifact ID")
    parser.add_argument("--mode", type=str, default="bregman", choices=["nominal", "euclidean", "bregman"], help="Controller mode")
    parser.add_argument("--coriolis", type=str.lower, default="lc", choices=["lc", "rb"], help="Coriolis factorization (lc or rb)")
    parser.add_argument("--duration", type=float, default=30.0, help="Simulation duration (seconds)")
    parser.add_argument("--gui", action="store_true", help="Launch interactive 3D PyBullet GUI")
    parser.add_argument("--speed", type=float, default=1.0, help="GUI playback speed multiplier (e.g. 1.0 for real-time, 2.0 for 2x)")
    parser.add_argument("--no-pacing", action="store_true", help="Disable wall-clock real-time pacing")
    parser.add_argument("--ground", type=str, default="arena", choices=["arena", "grid", "plane"], help="Ground visual style")
    parser.add_argument("--gates", type=str, default=None, choices=["lemniscate", "ratm", "auto", "none"], help="Racing gates mode")
    parser.add_argument("--drone-type", type=str, default="pybullet_drones", choices=["pybullet_drones", "hexacopter"], help="Visual drone model")
    parser.add_argument("--cam-mode", type=str, default="chase", choices=["chase", "fpv", "overview", "free"], help="Initial camera mode")
    parser.add_argument("--ground-z", type=float, default=0.0, help="Floor Z elevation (default: 0.0)")
    parser.add_argument("--osd", action="store_true", default=False, help="Enable FPV OSD HUD overlay (disabled by default)")
    parser.add_argument("--no-osd", action="store_true", help="Explicitly disable FPV OSD HUD overlay")
    parser.add_argument("--save-figures", action="store_true", default=True, help="Export paper figures")
    parser.add_argument("--output-dir", type=str, default=None, help="Custom output directory")
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

    # Determine ground_z (default to 0.0 for arena floor alignment)
    ground_z = float(args.ground_z)
    enable_osd = bool(args.osd and not args.no_osd)

    print("=" * 60)
    print(f"Running AGC Simulation: Mode = {args.mode}, Coriolis = {args.coriolis}")
    print(f"Trajectory = {args.replay_id}, Duration = {args.duration} s, GUI = {args.gui}, Speed = {args.speed}x")
    if args.gui:
        print(f"Visuals: Ground = {args.ground}, Gates = {args.gates or 'auto'}, Cam = {args.cam_mode}, OSD = {enable_osd}")
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
        ground_z=ground_z,
        ground_style=args.ground,
        gates_mode=args.gates,
        cam_mode=args.cam_mode,
        enable_osd=enable_osd,
        drone_type=args.drone_type,
    )

    run, failure = run_scenario(scenario)

    if args.output_dir is not None:
        results_dir = Path(args.output_dir)
    else:
        results_dir = default_simulation_results_root(
            str(REPO_ROOT), inplace_save=args.inplace_save
        ) / f"{args.mode}_{args.coriolis}"
    os.makedirs(results_dir, exist_ok=True)

    if failure is None:
        metrics = compute_metrics(run)
        print("\nSimulation Succeeded!")
        print(f"Position RMSE:           {metrics['positionRMSE']:.4f} m")
        print(f"Attitude RMSE:           {metrics['attitudeRMSE']:.4f} rad")
        print(f"Linear Velocity RMSE:    {metrics['linearVelocityRMSE']:.4f} m/s")
        print(f"Angular Velocity RMSE:   {metrics['angularVelocityRMSE']:.4f} rad/s")
        print(f"Max Position Error:      {metrics['maxPositionError']:.4f} m")
        print(f"Force RMS:               {metrics['forceRMS']:.4f} N")
        print(f"Torque RMS:              {metrics['torqueRMS']:.4f} N m")
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
        save_run(str(results_dir), run, metrics, scenario, failure=failure)


if __name__ == "__main__":
    main()
