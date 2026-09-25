"""Flat, paper-facing figure exports for the experiment runner."""

from pathlib import Path
from typing import Any, Dict, Iterable, Optional
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from .diagnostics import derive_diagnostics


def _save(fig: plt.Figure, output_dir: Path, name: str) -> str:
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / f"{name}.png"
    fig.savefig(path, dpi=300, bbox_inches="tight")
    plt.close(fig)
    return str(path)


def _release(ax: plt.Axes, release_time: Optional[float]) -> None:
    if release_time is not None:
        ax.axvline(release_time, color="black", linestyle=":", linewidth=1.0, label="payload release")


def export_adaptive_figures(
    runs: Dict[str, Dict[str, Any]],
    scenarios: Dict[str, dict],
    output_dir: str,
    release_time: float = 10.0,
) -> list[str]:
    """Export flat adaptive tracking, parameter, and wrench figures."""
    out = Path(output_dir)
    diagnostics = {name: derive_diagnostics(run, scenarios[name]) for name, run in runs.items()}
    exported = []

    fig, axes = plt.subplots(4, 1, figsize=(9, 11), sharex=True)
    for name, d in diagnostics.items():
        t = runs[name]["t"]
        axes[0].plot(t, d["positionError"], label=name)
        axes[1].plot(t, np.degrees(d["attitudeError"]), label=name)
        axes[2].plot(t, runs[name]["Psi"], label=name)
        axes[3].plot(t, d["slidingNorm"], label=name)
    labels = [("position error (m)", "Position error"), ("attitude error (deg)", "Geodesic attitude error"), ("Psi", "Configuration potential"), ("||s||", "Composite tracking error")]
    for ax, (ylabel, title) in zip(axes, labels):
        ax.set_ylabel(ylabel); ax.set_title(title); ax.grid(True, linestyle=":"); _release(ax, release_time)
    axes[-1].set_xlabel("time (s)"); axes[0].legend(loc="best")
    exported.append(_save(fig, out, "adaptive_tracking_overview"))

    fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    for name, d in diagnostics.items():
        t = runs[name]["t"]
        mask = t >= release_time
        axes[0].plot(t[mask] - release_time, d["positionError"][mask], label=name)
        axes[1].plot(t[mask] - release_time, d["attitudeError"][mask] * 180.0 / np.pi, label=name)
    axes[0].set_ylabel("position error (m)"); axes[1].set_ylabel("attitude error (deg)"); axes[1].set_xlabel("time since release (s)")
    for ax in axes: ax.grid(True, linestyle=":")
    axes[0].legend(loc="best")
    exported.append(_save(fig, out, "adaptive_tracking_release_zoom"))

    fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    for name, run in runs.items():
        t = run["t"]; wrench = run["wrench"]
        axes[0].plot(t, np.linalg.norm(wrench[:, 3:6], axis=1), label=name)
        axes[1].plot(t, np.linalg.norm(wrench[:, :3], axis=1), label=name)
    axes[0].set_ylabel("force norm (N)"); axes[1].set_ylabel("torque norm (N m)"); axes[1].set_xlabel("time (s)")
    for ax in axes: ax.grid(True, linestyle=":"); _release(ax, release_time)
    axes[0].legend(loc="best")
    exported.append(_save(fig, out, "adaptive_tracking_wrench"))

    fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    for name, d in diagnostics.items():
        axes[0].plot(runs[name]["t"], d["estimatePi"][:, 0], label=f"{name} estimate")
        axes[1].plot(runs[name]["t"], d["pseudoMargin"], label=name)
    axes[0].set_ylabel("mass (kg)"); axes[1].set_ylabel("min eig(Jhat)"); axes[1].set_xlabel("time (s)")
    for ax in axes: ax.grid(True, linestyle=":"); _release(ax, release_time)
    axes[0].legend(loc="best")
    exported.append(_save(fig, out, "physical_consistency"))
    return exported


def export_nominal_figures(
    run: Dict[str, Any],
    scenario: dict,
    output_dir: str,
    reaching_bound: Optional[float] = None,
    connection_residual: Optional[Iterable[float]] = None,
) -> list[str]:
    out = Path(output_dir)
    exported = []
    d = derive_diagnostics(run, scenario)
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(run["t"], d["slidingNorm"], label=r"$||s||$")
    ax.set(xlabel="time (s)", ylabel=r"$||s||$", title="Nominal finite-time reaching")
    ax.grid(True, linestyle=":")
    if reaching_bound is not None:
        ax.axvline(reaching_bound, color="tab:red", linestyle="--", label="theoretical bound")
    ax.legend(); exported.append(_save(fig, out, "nominal_reaching"))
    if connection_residual is not None:
        fig, ax = plt.subplots(figsize=(9, 5))
        ax.plot(run["t"][:len(connection_residual)], connection_residual)
        ax.set(xlabel="time (s)", ylabel="identity residual norm", title="LC/RB connection identity residual")
        ax.grid(True, linestyle=":"); exported.append(_save(fig, out, "connection_identity_residual"))
    return exported
