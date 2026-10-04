"""Small nominal tracking figure exporters."""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def export_run_figures(run: dict, output_dir: str):
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    t = np.asarray(run["t"])
    position_error = np.linalg.norm(run["H"][:, :3, 3] - run["Hdesired"][:, :3, 3], axis=1)
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(t, position_error, label="position error")
    ax.set(xlabel="time (s)", ylabel="position error (m)")
    ax.grid(True, linestyle=":")
    ax.legend()
    fig.tight_layout()
    fig.savefig(output / "tracking_position.png", dpi=200)
    plt.close(fig)
    return [output / "tracking_position.png"]


def export_suite_comparison_figures(*args, **kwargs):
    """Compatibility wrapper for nominal batch results."""
    return []
