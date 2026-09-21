"""Publication-quality figure generation using Matplotlib with suite-level comparisons and release-relative views."""

import os
from pathlib import Path
from typing import Dict, Any, List, Optional
import numpy as np

# Ensure headless execution
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from ..io.persistence import load_run


def _find_payload_release_time(run: Dict[str, Any]) -> Optional[float]:
    """Determine payload release time from metadata or active parameter transition."""
    if "metadata" in run and isinstance(run["metadata"], dict):
        p_drop = run["metadata"].get("payloadDrop")
        if p_drop and "releaseTime" in p_drop:
            return float(p_drop["releaseTime"])

    if "activePlantPi" in run and len(run["activePlantPi"]) > 1:
        masses = run["activePlantPi"][:, 0]
        diffs = np.where(np.abs(np.diff(masses)) > 0.01)[0]
        if len(diffs) > 0:
            idx = diffs[0] + 1
            if idx < len(run["t"]):
                return float(run["t"][idx])

    return None


def _render_run_figures(run: Dict[str, Any], target_dir: str, title_suffix: str = "", release_t: Optional[float] = None) -> List[str]:
    """Render and save tracking and adaptation figures for a given dataset slice."""
    os.makedirs(target_dir, exist_ok=True)
    t = run["t"]
    n = len(t)
    if n == 0:
        return []

    generated = []

    pos_err = np.zeros((n, 3))
    att_err = np.zeros(n)
    for k in range(n):
        pos_err[k] = run["H"][k, 0:3, 3] - run["Hdesired"][k, 0:3, 3]
        Re = run["Hdesired"][k, 0:3, 0:3].T @ run["H"][k, 0:3, 0:3]
        cos_th = np.clip((np.trace(Re) - 1.0) / 2.0, -1.0, 1.0)
        att_err[k] = np.arccos(cos_th)

    # 1. Tracking Performance Figure
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), sharex=True)

    axes[0, 0].plot(t, pos_err[:, 0], label=r"$e_x$", color="#1f77b4")
    axes[0, 0].plot(t, pos_err[:, 1], label=r"$e_y$", color="#ff7f0e")
    axes[0, 0].plot(t, pos_err[:, 2], label=r"$e_z$", color="#2ca02c")
    if release_t is not None and release_t >= t[0] and release_t <= t[-1]:
        axes[0, 0].axvline(release_t, color="gray", linestyle="--", alpha=0.7, label="Payload Drop")
    axes[0, 0].set_ylabel("Position Error (m)")
    axes[0, 0].set_title(f"Position Tracking Error{title_suffix}")
    axes[0, 0].grid(True, linestyle=":", alpha=0.6)
    axes[0, 0].legend(loc="upper right")

    axes[0, 1].plot(t, att_err, color="#d62728", label=r"$\|\tilde{R}\|$")
    if release_t is not None and release_t >= t[0] and release_t <= t[-1]:
        axes[0, 1].axvline(release_t, color="gray", linestyle="--", alpha=0.7)
    axes[0, 1].set_ylabel("Attitude Error (rad)")
    axes[0, 1].set_title(f"Attitude Tracking Error{title_suffix}")
    axes[0, 1].grid(True, linestyle=":", alpha=0.6)
    axes[0, 1].legend(loc="upper right")

    s_norm = np.linalg.norm(run["s"], axis=1)
    axes[1, 0].plot(t, s_norm, color="#9467bd", label=r"$\|s\|$")
    if release_t is not None and release_t >= t[0] and release_t <= t[-1]:
        axes[1, 0].axvline(release_t, color="gray", linestyle="--", alpha=0.7)
    axes[1, 0].set_xlabel("Time (s)")
    axes[1, 0].set_ylabel(r"$\|s\|$")
    axes[1, 0].set_title(f"Sliding Variable Norm{title_suffix}")
    axes[1, 0].grid(True, linestyle=":", alpha=0.6)
    axes[1, 0].legend(loc="upper right")

    axes[1, 1].plot(t, run["Psi"], color="#8c564b", label=r"$\Psi$")
    if release_t is not None and release_t >= t[0] and release_t <= t[-1]:
        axes[1, 1].axvline(release_t, color="gray", linestyle="--", alpha=0.7)
    axes[1, 1].set_xlabel("Time (s)")
    axes[1, 1].set_ylabel(r"$\Psi$")
    axes[1, 1].set_title(f"Configuration Potential{title_suffix}")
    axes[1, 1].grid(True, linestyle=":", alpha=0.6)
    axes[1, 1].legend(loc="upper right")

    plt.tight_layout()
    fig_path = os.path.join(target_dir, "tracking_performance.png")
    fig.savefig(fig_path, dpi=300)
    plt.close(fig)
    generated.append(fig_path)

    # 2. Adaptation Figure
    if "estimatePi" in run and "activePlantPi" in run and len(run["estimatePi"]) == n:
        fig_adapt, axes_ad = plt.subplots(2, 1, figsize=(9, 6), sharex=True)

        axes_ad[0].plot(t, run["estimatePi"][:, 0], label=r"$\hat{m}(t)$", color="#1f77b4", linewidth=1.5)
        axes_ad[0].plot(t, run["activePlantPi"][:, 0], label=r"$m_{true}(t)$", color="black", linestyle="--", linewidth=1.5)
        if release_t is not None and release_t >= t[0] and release_t <= t[-1]:
            axes_ad[0].axvline(release_t, color="gray", linestyle=":", alpha=0.7)
        axes_ad[0].set_ylabel("Mass (kg)")
        axes_ad[0].set_title(f"Inertial Parameter Adaptation: Mass{title_suffix}")
        axes_ad[0].grid(True, linestyle=":", alpha=0.6)
        axes_ad[0].legend(loc="best")

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
        if release_t is not None and release_t >= t[0] and release_t <= t[-1]:
            axes_ad[1].axvline(release_t, color="gray", linestyle=":", alpha=0.7)
        axes_ad[1].set_xlabel("Time (s)")
        axes_ad[1].set_ylabel("Center of Mass (m)")
        axes_ad[1].set_title(f"Inertial Parameter Adaptation: Center of Mass{title_suffix}")
        axes_ad[1].grid(True, linestyle=":", alpha=0.6)
        axes_ad[1].legend(loc="best", ncol=3)

        plt.tight_layout()
        adapt_path = os.path.join(target_dir, "parameter_adaptation.png")
        fig_adapt.savefig(adapt_path, dpi=300)
        plt.close(fig_adapt)
        generated.append(adapt_path)

    return generated


def export_run_figures(run: Dict[str, Any], output_dir: str, include_relative: bool = True) -> List[str]:
    """Generate and save publication tracking and adaptation figures for a single run in total and release-relative views."""
    os.makedirs(output_dir, exist_ok=True)
    release_t = _find_payload_release_time(run)
    generated = []

    # 1. Direct output_dir and total_sim directory
    generated.extend(_render_run_figures(run, output_dir, release_t=release_t))
    total_sim_dir = os.path.join(output_dir, "figures", "total-sim")
    generated.extend(_render_run_figures(run, total_sim_dir, title_suffix=" (Total Simulation)", release_t=release_t))

    # 2. Release-relative view (from-drop)
    if include_relative and release_t is not None and release_t < run["t"][-1]:
        mask = run["t"] >= release_t
        if np.any(mask) and np.sum(mask) > 1:
            run_drop = {
                k: v[mask] if isinstance(v, np.ndarray) and v.ndim > 0 and len(v) == len(run["t"]) else v
                for k, v in run.items()
            }
            from_drop_dir = os.path.join(output_dir, "figures", "from-drop")
            generated.extend(_render_run_figures(run_drop, from_drop_dir, title_suffix=" (Post-Payload-Drop)"))

    return generated


def _render_suite_comparisons(runs: Dict[str, Dict[str, Any]], target_dir: Path, title_suffix: str = "") -> List[str]:
    """Render C1 vs C2 and adaptation comparison plots for a subset of suite runs."""
    target_dir.mkdir(parents=True, exist_ok=True)
    generated = []

    # 1. C1 vs C2 Tracking Comparison
    fig, axes = plt.subplots(3, 2, figsize=(12, 9), sharex=True)
    controllers = ["nominal", "euclidean", "bregman"]

    for row_idx, ctrl in enumerate(controllers):
        c1_key = f"{ctrl}_c1"
        c2_key = f"{ctrl}_c2"

        if c1_key in runs:
            r1 = runs[c1_key]
            err1 = np.linalg.norm(r1["H"][:, 0:3, 3] - r1["Hdesired"][:, 0:3, 3], axis=1)
            axes[row_idx, 0].plot(r1["t"], err1, label=f"{ctrl.upper()} C1", color="#1f77b4", linewidth=1.5)
        if c2_key in runs:
            r2 = runs[c2_key]
            err2 = np.linalg.norm(r2["H"][:, 0:3, 3] - r2["Hdesired"][:, 0:3, 3], axis=1)
            axes[row_idx, 0].plot(r2["t"], err2, label=f"{ctrl.upper()} C2", color="#ff7f0e", linestyle="--", linewidth=1.5)

        axes[row_idx, 0].set_ylabel("Pos Error (m)")
        axes[row_idx, 0].set_title(f"{ctrl.capitalize()}: Position Error{title_suffix}")
        axes[row_idx, 0].grid(True, linestyle=":", alpha=0.6)
        axes[row_idx, 0].legend(loc="upper right")

        if c1_key in runs:
            axes[row_idx, 1].plot(runs[c1_key]["t"], np.linalg.norm(runs[c1_key]["s"], axis=1), label=f"{ctrl.upper()} C1", color="#1f77b4")
        if c2_key in runs:
            axes[row_idx, 1].plot(runs[c2_key]["t"], np.linalg.norm(runs[c2_key]["s"], axis=1), label=f"{ctrl.upper()} C2", color="#ff7f0e", linestyle="--")

        axes[row_idx, 1].set_ylabel(r"$\|s\|$")
        axes[row_idx, 1].set_title(f"{ctrl.capitalize()}: Sliding Variable{title_suffix}")
        axes[row_idx, 1].grid(True, linestyle=":", alpha=0.6)
        axes[row_idx, 1].legend(loc="upper right")

    axes[2, 0].set_xlabel("Time (s)")
    axes[2, 1].set_xlabel("Time (s)")
    plt.tight_layout()
    c1_c2_path = target_dir / "c1_vs_c2_comparison.png"
    fig.savefig(c1_c2_path, dpi=300)
    plt.close(fig)
    generated.append(str(c1_c2_path))

    # 2. Adaptation Comparison (Euclidean vs Bregman)
    if "euclidean_c1" in runs and "bregman_c1" in runs:
        fig_ad, axes_ad = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
        r_euc = runs["euclidean_c1"]
        r_breg = runs["bregman_c1"]

        axes_ad[0].plot(r_euc["t"], r_euc["estimatePi"][:, 0], label="Euclidean C1", color="#ff7f0e", linewidth=1.5)
        axes_ad[0].plot(r_breg["t"], r_breg["estimatePi"][:, 0], label="Bregman C1", color="#2ca02c", linewidth=1.5)
        axes_ad[0].plot(r_breg["t"], r_breg["activePlantPi"][:, 0], label="True Mass", color="black", linestyle="--", linewidth=1.5)
        axes_ad[0].set_ylabel("Mass (kg)")
        axes_ad[0].set_title(f"Mass Adaptation Comparison: Euclidean vs Bregman{title_suffix}")
        axes_ad[0].grid(True, linestyle=":", alpha=0.6)
        axes_ad[0].legend(loc="best")

        if "minPseudoEigenvalue" in r_breg and len(r_breg["minPseudoEigenvalue"]) == len(r_breg["t"]):
            axes_ad[1].plot(r_breg["t"], r_breg["minPseudoEigenvalue"], label=r"$\lambda_{\min}(J_{Bregman})$", color="#2ca02c")
            axes_ad[1].axhline(0.0, color="red", linestyle=":", label="SPD Boundary")
            axes_ad[1].set_ylabel(r"$\lambda_{\min}$")
            axes_ad[1].set_title("Bregman Pseudo-Inertia Positive-Definiteness Metric")
            axes_ad[1].grid(True, linestyle=":", alpha=0.6)
            axes_ad[1].legend(loc="best")

        axes_ad[1].set_xlabel("Time (s)")
        plt.tight_layout()
        ad_comp_path = target_dir / "adaptation_comparison.png"
        fig_ad.savefig(ad_comp_path, dpi=300)
        plt.close(fig_ad)
        generated.append(str(ad_comp_path))

    return generated


def export_suite_comparison_figures(suite_dir: str) -> List[str]:
    """Generate suite-level comparison figures in root, total-sim, and from-drop directories."""
    p_suite = Path(suite_dir)
    variants = ["nominal_c1", "nominal_c2", "euclidean_c1", "euclidean_c2", "bregman_c1", "bregman_c2"]
    runs = {}

    for var in variants:
        var_dir = p_suite / var
        if (var_dir / "run.npz").is_file():
            try:
                runs[var] = load_run(str(var_dir))
            except Exception:
                pass

    if not runs:
        return []

    generated = []

    # 1. Root suite figures & total-sim comparisons
    generated.extend(_render_suite_comparisons(runs, p_suite))
    generated.extend(_render_suite_comparisons(runs, p_suite / "comparisons" / "total-sim", title_suffix=" (Total Simulation)"))

    # 2. Release-relative suite comparisons (from-drop)
    # Find release time from available runs
    release_times = [_find_payload_release_time(r) for r in runs.values()]
    valid_releases = [rt for rt in release_times if rt is not None]
    if valid_releases:
        ref_release = valid_releases[0]
        runs_drop = {}
        for k, r in runs.items():
            mask = r["t"] >= ref_release
            if np.any(mask) and np.sum(mask) > 1:
                runs_drop[k] = {
                    key: val[mask] if isinstance(val, np.ndarray) and val.ndim > 0 and len(val) == len(r["t"]) else val
                    for key, val in r.items()
                }
        if runs_drop:
            generated.extend(_render_suite_comparisons(runs_drop, p_suite / "comparisons" / "from-drop", title_suffix=" (Post-Payload-Drop)"))

    return generated
