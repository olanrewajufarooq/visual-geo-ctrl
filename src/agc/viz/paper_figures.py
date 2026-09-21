"""Publication-quality figure generation using Matplotlib."""

import os
from typing import Dict, Any
import numpy as np
import matplotlib.pyplot as plt


def export_run_figures(run: Dict[str, Any], output_dir: str):
    """Generate and save publication tracking and adaptation figures for a single run."""
    os.makedirs(output_dir, exist_ok=True)
    t = run["t"]
    n = len(t)

    pos_err = np.zeros((n, 3))
    att_err = np.zeros(n)
    for k in range(n):
        pos_err[k] = run["H"][k, 0:3, 3] - run["Hdesired"][k, 0:3, 3]
        Re = run["Hdesired"][k, 0:3, 0:3].T @ run["H"][k, 0:3, 0:3]
        cos_th = np.clip((np.trace(Re) - 1.0) / 2.0, -1.0, 1.0)
        att_err[k] = np.arccos(cos_th)

    # 1. Tracking Performance Figure
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), sharex=True)

    # Position Error
    axes[0, 0].plot(t, pos_err[:, 0], label=r"$e_x$", color="#1f77b4")
    axes[0, 0].plot(t, pos_err[:, 1], label=r"$e_y$", color="#ff7f0e")
    axes[0, 0].plot(t, pos_err[:, 2], label=r"$e_z$", color="#2ca02c")
    axes[0, 0].axvline(10.0, color="gray", linestyle="--", alpha=0.7, label="Payload Drop")
    axes[0, 0].set_ylabel("Position Error (m)")
    axes[0, 0].set_title("Position Tracking Error")
    axes[0, 0].grid(True, linestyle=":", alpha=0.6)
    axes[0, 0].legend(loc="upper right")

    # Attitude Error
    axes[0, 1].plot(t, att_err, color="#d62728", label=r"$\|\tilde{R}\|$")
    axes[0, 1].axvline(10.0, color="gray", linestyle="--", alpha=0.7)
    axes[0, 1].set_ylabel("Attitude Error (rad)")
    axes[0, 1].set_title("Attitude Tracking Error")
    axes[0, 1].grid(True, linestyle=":", alpha=0.6)
    axes[0, 1].legend(loc="upper right")

    # Sliding variable norm
    s_norm = np.linalg.norm(run["s"], axis=1)
    axes[1, 0].plot(t, s_norm, color="#9467bd", label=r"$\|s\|$")
    axes[1, 0].axvline(10.0, color="gray", linestyle="--", alpha=0.7)
    axes[1, 0].set_xlabel("Time (s)")
    axes[1, 0].set_ylabel(r"$\|s\|$")
    axes[1, 0].set_title("Sliding Variable Norm")
    axes[1, 0].grid(True, linestyle=":", alpha=0.6)
    axes[1, 0].legend(loc="upper right")

    # Configuration Potential Psi
    axes[1, 1].plot(t, run["Psi"], color="#8c564b", label=r"$\Psi$")
    axes[1, 1].axvline(10.0, color="gray", linestyle="--", alpha=0.7)
    axes[1, 1].set_xlabel("Time (s)")
    axes[1, 1].set_ylabel(r"$\Psi$")
    axes[1, 1].set_title("Configuration Potential")
    axes[1, 1].grid(True, linestyle=":", alpha=0.6)
    axes[1, 1].legend(loc="upper right")

    plt.tight_layout()
    fig_path = os.path.join(output_dir, "tracking_performance.png")
    fig.savefig(fig_path, dpi=300)
    plt.close(fig)

    # 2. Adaptation Figure
    if "estimatePi" in run and "activePlantPi" in run:
        fig_adapt, axes_ad = plt.subplots(2, 1, figsize=(9, 6), sharex=True)

        # Mass estimation
        axes_ad[0].plot(t, run["estimatePi"][:, 0], label=r"$\hat{m}(t)$", color="#1f77b4", linewidth=1.5)
        axes_ad[0].plot(t, run["activePlantPi"][:, 0], label=r"$m_{true}(t)$", color="black", linestyle="--", linewidth=1.5)
        axes_ad[0].axvline(10.0, color="gray", linestyle=":", alpha=0.7)
        axes_ad[0].set_ylabel("Mass (kg)")
        axes_ad[0].set_title("Inertial Parameter Adaptation: Mass")
        axes_ad[0].grid(True, linestyle=":", alpha=0.6)
        axes_ad[0].legend(loc="best")

        # Center of mass estimation
        est_m = np.maximum(run["estimatePi"][:, 0:1], 1e-6)
        true_m = np.maximum(run["activePlantPi"][:, 0:1], 1e-6)
        est_cog = run["estimatePi"][:, 1:4] / est_m
        true_cog = run["activePlantPi"][:, 1:4] / true_m

        axes_ad[1].plot(t, est_cog[:, 0], label=r"$\hat{r}_x$", color="#ff7f0e")
        axes_ad[1].plot(t, true_cog[:, 0], label=r"$r_{x, true}$", color="#ff7f0e", linestyle="--")
        axes_ad[1].plot(t, est_cog[:, 1], label=r"$\hat{r}_y$", color="#2ca02c")
        axes_ad[1].plot(t, true_cog[:, 1], label=r"$r_{y, true}$", color="#2ca02c", linestyle="--")
        axes_ad[1].plot(t, est_cog[:, 2], label=r"$\hat{r}_z$", color="#d62728")
        axes_ad[1].plot(t, true_cog[:, 2], label=r"$r_{z, true}$", color="#d62728", linestyle="--")
        axes_ad[1].axvline(10.0, color="gray", linestyle=":", alpha=0.7)
        axes_ad[1].set_xlabel("Time (s)")
        axes_ad[1].set_ylabel("Center of Mass (m)")
        axes_ad[1].set_title("Inertial Parameter Adaptation: Center of Mass")
        axes_ad[1].grid(True, linestyle=":", alpha=0.6)
        axes_ad[1].legend(loc="best", ncol=3)

        plt.tight_layout()
        adapt_path = os.path.join(output_dir, "parameter_adaptation.png")
        fig_adapt.savefig(adapt_path, dpi=300)
        plt.close(fig_adapt)
