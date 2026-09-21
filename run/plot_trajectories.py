"""Plot and inspect recorded reference flight trajectories."""

import sys
import argparse
from pathlib import Path

# Ensure headless plotting
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "src"))

from agc.sim.replay_trajectory import ReplayTrajectory


def main():
    parser = argparse.ArgumentParser(description="Plot and inspect recorded reference flight trajectories.")
    parser.add_argument(
        "--replay-id",
        type=str,
        default="lemniscate_01_auto",
        help="Trajectory benchmark ID (e.g. lemniscate_01_auto, ellipse_01_auto, RATM_01_auto)",
    )
    parser.add_argument(
        "--output",
        type=str,
        default=None,
        help="Path to save output inspection image",
    )
    args = parser.parse_args()

    traj_id = args.replay_id
    traj_file = REPO_ROOT / "trajectories" / "processed" / f"{traj_id}.mat"
    if not traj_file.is_file():
        print(f"Error: Trajectory file not found: {traj_file}")
        sys.exit(1)

    sampler = ReplayTrajectory(str(traj_file))

    print(f"Loaded trajectory: {traj_id}")
    print(f"Time span: {sampler.t[0]:.2f} s to {sampler.t[-1]:.2f} s ({len(sampler.t)} samples)")

    fig = plt.figure(figsize=(12, 5))
    ax1 = fig.add_subplot(1, 2, 1, projection="3d")
    ax1.plot(sampler.p[:, 0], sampler.p[:, 1], sampler.p[:, 2], label="Desired Position", color="#1f77b4")
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.set_zlabel("Z (m)")
    ax1.set_title(f"3D Flight Path: {traj_id}")
    ax1.legend()

    ax2 = fig.add_subplot(1, 2, 2)
    ax2.plot(sampler.t, sampler.v_b[:, 0], label=r"$v_x$ (body)")
    ax2.plot(sampler.t, sampler.v_b[:, 1], label=r"$v_y$ (body)")
    ax2.plot(sampler.t, sampler.v_b[:, 2], label=r"$v_z$ (body)")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Linear Velocity (m/s)")
    ax2.set_title("Body Linear Velocities")
    ax2.grid(True, linestyle=":", alpha=0.6)
    ax2.legend()

    plt.tight_layout()
    if args.output is not None:
        out_file = Path(args.output)
    else:
        out_file = REPO_ROOT / "results" / f"trajectory_{traj_id}.png"

    out_file.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(out_file, dpi=300)
    print(f"Saved inspection plot to {out_file}")
    plt.close()


if __name__ == "__main__":
    main()
