"""Plot and inspect recorded reference flight trajectories.

By default all three canonical trajectories are plotted and saved to
``results/trajectories/``.  Each figure mirrors the MATLAB tiled layout:

  ┌────────────────┬──────────────────────┬──────────────────────────┐
  │                │  position (m)        │  body lin. velocity m/s  │
  │  3-D path      ├──────────────────────┼──────────────────────────┤
  │  (3 rows tall) │  body lin. accel m/s²│  body ang. velocity rad/s│
  │                ├──────────────────────┼──────────────────────────┤
  │                │  body ang. accel r/s²│                          │
  └────────────────┴──────────────────────┴──────────────────────────┘

Usage
-----
  # plot all three trajectories (default):
  python run/plot_trajectories.py

  # plot a single trajectory:
  python run/plot_trajectories.py --ids lemniscate_01_auto

  # override output directory:
  python run/plot_trajectories.py --output-dir path/to/dir
"""

import sys
import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "src"))

from vgc.sim.replay_trajectory import ReplayTrajectory
from vgc.viz.plot_style import COMPONENT_COLORS, limit_for, plot_time_series

# ── canonical ordering matches manifest.json ──────────────────────────────────
ALL_IDS = ["lemniscate_01_auto", "lemniscate_02_auto", "lemniscate_03_auto", "lemniscate_04_auto"]

# Pretty labels used in panel titles
CHANNEL_LABELS = {
    "position":            ("x", "y", "z"),
    "body lin. velocity":  ("$v_x$", "$v_y$", "$v_z$"),
    "body lin. accel.":    ("$a_x$", "$a_y$", "$a_z$"),
    "body ang. velocity":  (r"$\omega_x$", r"$\omega_y$", r"$\omega_z$"),
    "body ang. accel.":    (r"$\alpha_x$", r"$\alpha_y$", r"$\alpha_z$"),
    "orientation (ZYX)":   ("roll", "pitch", "yaw"),
}
CHANNEL_UNITS = {
    "position":            "m",
    "body lin. velocity":  "m/s",
    "body lin. accel.":    r"m/s^2",
    "body ang. velocity":  "rad/s",
    "body ang. accel.":    r"rad/s^2",
    "orientation (ZYX)":   "deg",
}


def _rotm_to_euler_zyx_deg(R: np.ndarray) -> np.ndarray:
    """Convert rotation matrices (3,3,N) to ZYX Euler angles (N,3) in degrees.

    Returns columns [roll (phi), pitch (theta), yaw (psi)] in degrees.
    Convention: R = Rz(psi) @ Ry(theta) @ Rx(phi), body-to-world.
    """
    N = R.shape[2]
    euler = np.zeros((N, 3))
    for k in range(N):
        Rk = R[:, :, k]
        sin_theta = np.clip(-Rk[2, 0], -1.0, 1.0)
        theta = np.arcsin(sin_theta)
        phi   = np.arctan2(Rk[2, 1], Rk[2, 2])
        psi   = np.arctan2(Rk[1, 0], Rk[0, 0])
        euler[k] = [np.degrees(phi), np.degrees(theta), np.degrees(psi)]
    return euler


def _plot_one(traj_id: str, out_dir: Path) -> None:
    """Produce the 3x3 inspection figure for a single trajectory ID."""
    processed_dir = REPO_ROOT / "trajectories" / "processed"
    npz_file = processed_dir / f"{traj_id}.npz"
    traj_file = npz_file if npz_file.is_file() else (processed_dir / f"{traj_id}.mat")
    if not traj_file.is_file():
        print(f"  [SKIP]  Trajectory file not found: {traj_file}")
        return

    s = ReplayTrajectory(str(traj_file))

    print(f"  Loaded {traj_id}  |  {s.t[0]:.2f} s -> {s.t[-1]:.2f} s  ({len(s.t)} samples)")

    # Derive orientation from stored rotation matrix R (3,3,N)
    euler_deg = _rotm_to_euler_zyx_deg(s.R)

    # ── figure + layout ───────────────────────────────────────────────────────
    fig = plt.figure(figsize=(14, 8))
    fig.suptitle(f"Preprocessed replay: {traj_id}", fontsize=13, fontweight="bold")

    # 3-D path occupies column 0, all 3 rows (subplot indices 1, 4, 7)
    ax3d = fig.add_subplot(3, 3, (1, 7), projection="3d")

    # Six panels fill the right two columns (positions 2,3,5,6,8,9).
    # Layout: row 0 → position | attitude
    #         row 1 → lin. velocity | ang. velocity
    #         row 2 → lin. accel.   | ang. accel.
    panel_positions = [2, 3, 5, 6, 8, 9]
    channels = [
        ("position",           s.p),           # col 1, row 0
        ("orientation (ZYX)",  euler_deg),     # col 2, row 0
        ("body lin. velocity", s.v_b),         # col 1, row 1
        ("body ang. velocity", s.omega_b),     # col 2, row 1
        ("body lin. accel.",   s.a_b),         # col 1, row 2
        ("body ang. accel.",   s.alpha_b),     # col 2, row 2
    ]

    # ── 3-D path ──────────────────────────────────────────────────────────────
    ax3d.plot(s.p[:, 0], s.p[:, 1], s.p[:, 2], "k-", linewidth=1.4, label="path")
    ax3d.plot(*s.p[0],   marker="o", color=COMPONENT_COLORS[0], markersize=7, label="start")
    ax3d.plot(*s.p[-1],  marker="d", color=COMPONENT_COLORS[2], markersize=7, label="end")
    ax3d.set_xlim(*limit_for("position", 0)); ax3d.set_ylim(*limit_for("position", 1)); ax3d.set_zlim(*limit_for("position", 2))
    ax3d.set_xlabel("x (m)")
    ax3d.set_ylabel("y (m)")
    ax3d.set_zlabel("z (m)")
    ax3d.set_title("3-D Flight Path", fontsize=10)
    ax3d.legend(loc="best", fontsize=8)
    ax3d.grid(True)

    # ── kinematic + orientation panels ───────────────────────────────────────
    for grid_pos, (title, data) in zip(panel_positions, channels):
        ax = fig.add_subplot(3, 3, grid_pos)
        labels = CHANNEL_LABELS[title]
        family = {
            "position": "position",
            "orientation (ZYX)": "orientation",
            "body lin. velocity": "linear_velocity",
            "body ang. velocity": "angular_velocity",
            "body lin. accel.": "linear_acceleration",
            "body ang. accel.": "angular_acceleration",
        }[title]
        plot_time_series(ax, s.t, [data[:, j] for j in range(data.shape[1])], labels, family, (float(s.t[0]), float(s.t[-1])))
        ax.set_title(title.capitalize(), fontsize=9)
        ax.set_ylabel(CHANNEL_UNITS[title], fontsize=8)
        ax.set_xlabel("time (s)", fontsize=8)
        ax.legend(loc="best", fontsize=7, ncol=1)
        ax.grid(True, linestyle=":", alpha=0.6)
        ax.tick_params(labelsize=7)

    plt.tight_layout()

    # ── save ──────────────────────────────────────────────────────────────────
    out_dir.mkdir(parents=True, exist_ok=True)
    png_path = out_dir / f"{traj_id}_trajectory.png"
    plt.savefig(png_path, dpi=300, bbox_inches="tight")
    print(f"  Saved -> {png_path}")
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Plot all canonical reference trajectories into results/trajectories/.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--ids",
        nargs="+",
        metavar="ID",
        default=ALL_IDS,
        help=(
            "Trajectory IDs to plot (default: all three canonical trajectories). "
            f"Choices: {ALL_IDS}"
        ),
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default=None,
        help="Directory to write output PNGs (default: results/trajectories/)",
    )
    args = parser.parse_args()

    out_dir = Path(args.output_dir) if args.output_dir else REPO_ROOT / "results" / "trajectories"

    print(f"Output directory: {out_dir}")
    print(f"Plotting {len(args.ids)} trajectory(ies): {args.ids}")
    print()

    for traj_id in args.ids:
        _plot_one(traj_id, out_dir)

    print("\nDone.")


if __name__ == "__main__":
    main()
