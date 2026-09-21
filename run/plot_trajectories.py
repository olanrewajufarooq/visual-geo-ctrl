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

from agc.sim.replay_trajectory import ReplayTrajectory

# ── canonical ordering matches manifest.json ──────────────────────────────────
ALL_IDS = ["ellipse_01_auto", "lemniscate_01_auto", "RATM_01_auto"]

# Pretty labels used in panel titles
CHANNEL_LABELS = {
    "position":            ("x", "y", "z"),
    "body lin. velocity":  ("$v_x$", "$v_y$", "$v_z$"),
    "body lin. accel.":    ("$a_x$", "$a_y$", "$a_z$"),
    "body ang. velocity":  (r"$\omega_x$", r"$\omega_y$", r"$\omega_z$"),
    "body ang. accel.":    (r"$\alpha_x$", r"$\alpha_y$", r"$\alpha_z$"),
}
CHANNEL_UNITS = {
    "position":            "m",
    "body lin. velocity":  "m/s",
    "body lin. accel.":    r"m/s²",
    "body ang. velocity":  "rad/s",
    "body ang. accel.":    r"rad/s²",
}


def _plot_one(traj_id: str, out_dir: Path) -> None:
    """Produce the 3×3 inspection figure for a single trajectory ID."""
    processed_dir = REPO_ROOT / "trajectories" / "processed"
    npz_file = processed_dir / f"{traj_id}.npz"
    traj_file = npz_file if npz_file.is_file() else (processed_dir / f"{traj_id}.mat")
    if not traj_file.is_file():
        print(f"  [SKIP]  Trajectory file not found: {traj_file}")
        return

    s = ReplayTrajectory(str(traj_file))

    print(f"  Loaded {traj_id}  |  {s.t[0]:.2f} s -> {s.t[-1]:.2f} s  ({len(s.t)} samples)")

    # ── figure + layout ───────────────────────────────────────────────────────
    fig = plt.figure(figsize=(14, 8))
    fig.suptitle(f"Preprocessed replay: {traj_id}", fontsize=13, fontweight="bold")

    # 3-D path occupies column 0, all 3 rows
    ax3d = fig.add_subplot(3, 3, (1, 7), projection="3d")

    # Five kinematic panels fill columns 1-2, rows 0-2 (positions 2,3,5,6,8,9
    # in 1-indexed grid — but we only have 5 panels so leave one blank or fill)
    panel_positions = [2, 3, 5, 6, 8]
    channels = [
        ("position",           s.p),
        ("body lin. velocity", s.v_b),
        ("body lin. accel.",   s.a_b),
        ("body ang. velocity", s.omega_b),
        ("body ang. accel.",   s.alpha_b),
    ]

    # ── 3-D path ──────────────────────────────────────────────────────────────
    ax3d.plot(s.p[:, 0], s.p[:, 1], s.p[:, 2], "k-", linewidth=1.4, label="path")
    ax3d.plot(*s.p[0],   "go", markersize=7, markerfacecolor="g", label="start")
    ax3d.plot(*s.p[-1],  "rd", markersize=7, markerfacecolor="r", label="end")
    ax3d.set_xlabel("x (m)")
    ax3d.set_ylabel("y (m)")
    ax3d.set_zlabel("z (m)")
    ax3d.set_title("3-D Flight Path", fontsize=10)
    ax3d.legend(loc="best", fontsize=8)
    ax3d.grid(True)

    # ── kinematic panels ──────────────────────────────────────────────────────
    for grid_pos, (title, data) in zip(panel_positions, channels):
        ax = fig.add_subplot(3, 3, grid_pos)
        labels = CHANNEL_LABELS[title]
        for j, lbl in enumerate(labels):
            ax.plot(s.t, data[:, j], linewidth=1.2, label=lbl)
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
