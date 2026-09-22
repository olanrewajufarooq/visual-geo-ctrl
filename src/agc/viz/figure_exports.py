"""PNG-only standalone and suite figure exports matching MATLAB coverage."""

from pathlib import Path
from typing import Any, Dict, List, Optional
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from .diagnostics import derive_diagnostics, slice_release_window
from ..io.persistence import load_run

BASE_FIGURES = [
    "trajectory_3d", "trajectory_xy", "altitude", "position", "attitude",
    "linear_velocity", "angular_velocity", "wrench_force", "wrench_torque",
    "sliding_norm", "potential", "transverse_energy",
]
ADAPTIVE_FIGURES = [
    "parameter_error", "pseudo_inertia_margin", "mass", "cog",
    "inertia_principal", "inertia_off_diagonal",
]


def _scenario_from_run(run: Dict[str, Any]) -> Dict[str, Any]:
    meta = run.get("metadata", {})
    controller = meta.get("controller", {})
    for key in ("KR", "Kxi", "Lambda", "gammaE"):
        if key in controller:
            controller[key] = np.asarray(controller[key], dtype=float)
    if "gammaB" in controller:
        controller["gammaB"] = float(controller["gammaB"])
    return {
        "plantPi": np.asarray(meta.get("plantPi", run.get("activePlantPi", [[0] * 10])[0]), dtype=float),
        "controller": controller,
    }


def _release_time(run: Dict[str, Any]) -> Optional[float]:
    drop = run.get("metadata", {}).get("payloadDrop")
    if drop and "releaseTime" in drop:
        return float(drop["releaseTime"])
    active = run.get("activePlantPi")
    if isinstance(active, np.ndarray) and len(active) > 1:
        changes = np.flatnonzero(np.abs(np.diff(active[:, 0])) > 0.01)
        if changes.size:
            return float(run["t"][changes[0] + 1])
    return None


def _save(fig, path: Path, visible: bool) -> Dict[str, Any]:
    # `visible` controls whether callers request interactive display; it must
    # never control the figure's artist visibility.  Hiding the Figure before
    # savefig produces a valid-looking, completely blank PNG in the default
    # headless mode (`visible=False`).
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=300, format="png")
    if visible:
        plt.show(block=False)
    plt.close(fig)
    return {"name": path.stem, "directory": str(path.parent), "files": {"png": str(path)}}


def _time_plot(t, series, labels, title, ylabel, path, release, visible):
    fig, ax = plt.subplots(figsize=(9, 5))
    for values, label in zip(series, labels):
        ax.plot(t, values, label=label)
    if release is not None and t[0] <= release <= t[-1]:
        ax.axvline(release, color="k", linestyle=":", label="payload release")
    ax.set(title=title, xlabel="time (s)", ylabel=ylabel); ax.grid(True, linestyle=":"); ax.legend(loc="best")
    return _save(fig, path, visible)


def _render_run(run, scenario, output_dir: Path, visible=False, suffix="") -> List[Dict[str, Any]]:
    d = derive_diagnostics(run, scenario)
    t = np.asarray(run["t"]); rel = _release_time(run)
    out = []
    def tp(name, series, labels, title, ylabel, log=False):
        values = [np.maximum(np.abs(v), 1e-12) for v in series] if log else series
        rec = _time_plot(t, values, labels, title + suffix, ylabel, output_dir / f"{name}.png", rel, visible)
        out.append(rec)
    # Trajectory views
    fig = plt.figure(figsize=(8, 6)); ax = fig.add_subplot(111, projection="3d")
    ax.plot(d["desiredPosition"][:, 0], d["desiredPosition"][:, 1], d["desiredPosition"][:, 2], "k--", label="desired")
    ax.plot(d["position"][:, 0], d["position"][:, 1], d["position"][:, 2], label="actual")
    ax.set(title="Trajectory" + suffix, xlabel="x (m)", ylabel="y (m)", zlabel="z (m)"); ax.grid(True); ax.legend()
    out.append(_save(fig, output_dir / "trajectory_3d.png", visible))
    fig, ax = plt.subplots(figsize=(8, 6)); ax.plot(d["desiredPosition"][:, 0], d["desiredPosition"][:, 1], "k--", label="desired"); ax.plot(d["position"][:, 0], d["position"][:, 1], label="actual"); ax.set(title="XY trajectory" + suffix, xlabel="x (m)", ylabel="y (m)"); ax.axis("equal"); ax.grid(True); ax.legend(); out.append(_save(fig, output_dir / "trajectory_xy.png", visible))
    tp("altitude", [d["position"][:, 2], d["desiredPosition"][:, 2]], ["actual", "desired"], "Altitude", "z (m)")
    tp("position", [d["position"][:, i] for i in range(3)] + [d["desiredPosition"][:, i] for i in range(3)], ["x", "y", "z", "x desired", "y desired", "z desired"], "Cartesian position", "p (m)")
    tp("attitude", [d["rpy"][:, i] for i in range(3)] + [d["desiredRpy"][:, i] for i in range(3)], ["roll", "pitch", "yaw", "roll desired", "pitch desired", "yaw desired"], "Attitude", "angle (rad)")
    tp("linear_velocity", [run["V"][:, i] for i in range(3, 6)] + [run["Vdesired"][:, i] for i in range(3, 6)], ["v_x", "v_y", "v_z", "v_x desired", "v_y desired", "v_z desired"], "Body linear velocity", "v (m/s)")
    tp("angular_velocity", [run["V"][:, i] for i in range(3)] + [run["Vdesired"][:, i] for i in range(3)], ["omega_x", "omega_y", "omega_z", "omega_x desired", "omega_y desired", "omega_z desired"], "Body angular velocity", "omega (rad/s)")
    tp("wrench_force", [run["wrench"][:, i] for i in range(3, 6)], ["F_x", "F_y", "F_z"], "Control force", "force (N)")
    tp("wrench_torque", [run["wrench"][:, i] for i in range(3)], ["tau_x", "tau_y", "tau_z"], "Control torque", "torque (Nm)")
    tp("sliding_norm", [d["slidingNorm"]], ["||s||"], "Sliding residual", "||s||", True)
    tp("potential", [run["Psi"]], ["Psi"], "Configuration potential", "Psi")
    tp("transverse_energy", [d["Vs"]], ["V_s"], "Transverse energy", "V_s", True)
    if str(run.get("mode", "")).lower() != "nominal":
        tp("parameter_error", [d["parameterError"]], ["normalized parameter error"], "Parameter error", "normalized error")
        tp("pseudo_inertia_margin", [d["pseudoMargin"]], ["min eig(Jhat)"], "Pseudo-inertia margin", "minimum eigenvalue")
        tp("mass", [d["estimatePi"][:, 0], d["truePi"][:, 0]], ["estimate", "true"], "Mass estimate", "mass (kg)")
        tp("cog", [d["cog"][:, i] for i in range(3)] + [d["trueCog"][:, i] for i in range(3)], ["rx", "ry", "rz", "rx true", "ry true", "rz true"], "Center of mass", "CoG (m)")
        tp("inertia_principal", [d["estimatePi"][:, i] for i in range(4, 7)] + [d["truePi"][:, i] for i in range(4, 7)], ["Ixx", "Iyy", "Izz", "Ixx true", "Iyy true", "Izz true"], "Principal inertia", "inertia (kg m^2)")
        tp("inertia_off_diagonal", [d["estimatePi"][:, i] for i in range(7, 10)] + [d["truePi"][:, i] for i in range(7, 10)], ["Ixy", "Ixz", "Iyz", "Ixy true", "Ixz true", "Iyz true"], "Off-diagonal inertia", "inertia (kg m^2)")
    return out


def export_run_figures(run: Dict[str, Any], output_dir: str, include_relative: bool = True, visible: bool = False) -> List[Dict[str, Any]]:
    scenario = _scenario_from_run(run)
    output = Path(output_dir)
    records = _render_run(run, scenario, output / "figures" / "total-sim", visible, " (Total Simulation)")
    release = _release_time(run)
    if include_relative and release is not None and release < float(run["t"][-1]):
        diagnostics = derive_diagnostics(run, scenario)
        run_drop, _ = slice_release_window(run, diagnostics, release)
        records.extend(_render_run(run_drop, scenario, output / "figures" / "from-drop", visible, " (Post-Payload-Drop)"))
    return records


def _load_suite(suite_dir: Path) -> Dict[str, Dict[str, Any]]:
    runs = {}
    for child in suite_dir.iterdir():
        if child.is_dir() and (child / "run.npz").is_file():
            runs[child.name] = load_run(str(child))
    return runs


def export_suite_comparison_figures(suite_dir: str, visible: bool = False, export_comparisons: bool = True) -> List[Dict[str, Any]]:
    """Export all available suite comparisons as 300-DPI PNGs."""
    if not export_comparisons:
        return []
    suite = Path(suite_dir); runs = _load_suite(suite); out = []
    controllers = ["nominal", "euclidean", "bregman"]
    for window_name, window_runs in [("total-sim", runs)]:
        root = suite / "comparisons" / window_name
        for ctrl in controllers:
            pairs = [window_runs.get(f"{ctrl}_c1"), window_runs.get(f"{ctrl}_c2")]
            if not all(pairs): continue
            fig3 = plt.figure(figsize=(8, 6)); ax3 = fig3.add_subplot(111, projection="3d")
            for run, label in zip(pairs, ("C1", "C2")):
                d = derive_diagnostics(run, _scenario_from_run(run)); ax3.plot(d["position"][:, 0], d["position"][:, 1], d["position"][:, 2], label=label)
            ax3.set(title=f"{ctrl} C1/C2 trajectory", xlabel="x (m)", ylabel="y (m)", zlabel="z (m)"); ax3.legend(); ax3.grid(True); out.append(_save(fig3, suite / "comparisons" / "total-sim" / f"{ctrl}_trajectory_c1_c2.png", visible))
            fig, axes = plt.subplots(3, 2, figsize=(12, 9), sharex=True)
            for row, key in enumerate(("positionError", "attitudeError", "slidingNorm")):
                for run, label in zip(pairs, ("C1", "C2")):
                    sc = _scenario_from_run(run); d = derive_diagnostics(run, sc); axes[row, 0].plot(run["t"], d[key], label=label)
                axes[row, 0].set_title(f"{ctrl} {key}"); axes[row, 0].legend(); axes[row, 0].grid(True, linestyle=":")
            # Add energy, wrench, and trajectory comparisons as separate MATLAB-equivalent claims.
            for name, key, ylabel in (("transverse_energy", "Vs", "V_s"), ("wrench_norm", "wrenchNorm", "||W||")):
                fig2, ax2 = plt.subplots(figsize=(9, 5))
                for run, label in zip(pairs, ("C1", "C2")):
                    sc = _scenario_from_run(run); d = derive_diagnostics(run, sc); vals = d[key] if key in d else d["wrenchNorm"]; ax2.plot(run["t"], vals, label=label)
                ax2.set(title=f"{ctrl} {name}", xlabel="time (s)", ylabel=ylabel); ax2.grid(True, linestyle=":"); ax2.legend(); out.append(_save(fig2, root / f"{ctrl}_{name}.png", visible))
            out.append(_save(fig, root / f"{ctrl}_c1_c2_comparison.png", visible))
    for form in ("c1", "c2"):
        e, b = runs.get(f"euclidean_{form}"), runs.get(f"bregman_{form}")
        if e is None or b is None: continue
        fields = (("position_error", "positionError", "Position error"), ("attitude_error", "attitudeError", "Attitude error"), ("sliding_norm", "slidingNorm", "Sliding norm"), ("parameter_error", "parameterError", "Parameter error"), ("pseudo_inertia_margin", "pseudoMargin", "Pseudo-inertia margin"))
        for name, key, title in fields:
            fig, ax = plt.subplots(figsize=(9, 5))
            for run, label in ((e, "Euclidean"), (b, "Bregman")):
                d = derive_diagnostics(run, _scenario_from_run(run)); ax.plot(run["t"], d[key], label=label)
            ax.set(title=f"Euclidean vs Bregman {title.lower()} ({form})", xlabel="time (s)", ylabel=title); ax.grid(True, linestyle=":"); ax.legend(); out.append(_save(fig, suite / "comparisons" / "euclidean_v_bregman" / f"{name}_{form}.png", visible))

    # Repeat the unit-consistent comparisons in a release-relative window.
    releases = [_release_time(run) for run in runs.values()]
    release = next((value for value in releases if value is not None), None)
    if release is not None:
        drop_runs = {}
        for key, run in runs.items():
            if release < float(run["t"][-1]):
                sc = _scenario_from_run(run); d = derive_diagnostics(run, sc); drop_runs[key], _ = slice_release_window(run, d, release)
        root = suite / "comparisons" / "from-drop"
        for ctrl in controllers:
            c1, c2 = drop_runs.get(f"{ctrl}_c1"), drop_runs.get(f"{ctrl}_c2")
            if c1 is None or c2 is None: continue
            for name, key in (("position_error", "positionError"), ("attitude_error", "attitudeError"), ("sliding_norm", "slidingNorm"), ("transverse_energy", "Vs"), ("wrench_norm", "wrenchNorm")):
                fig, ax = plt.subplots(figsize=(9, 5))
                for run, label in ((c1, "C1"), (c2, "C2")):
                    d = derive_diagnostics(run, _scenario_from_run(run)); ax.plot(run["t"], d[key], label=label)
                ax.set(title=f"{ctrl} {name} (Post-Payload-Drop)", xlabel="time (s)", ylabel=name); ax.grid(True, linestyle=":"); ax.legend(); out.append(_save(fig, root / f"{ctrl}_{name}.png", visible))
        for form in ("c1", "c2"):
            e, b = drop_runs.get(f"euclidean_{form}"), drop_runs.get(f"bregman_{form}")
            if e is None or b is None: continue
            for name, key in (("position_error", "positionError"), ("attitude_error", "attitudeError"), ("sliding_norm", "slidingNorm"), ("parameter_error", "parameterError"), ("pseudo_inertia_margin", "pseudoMargin")):
                fig, ax = plt.subplots(figsize=(9, 5))
                for run, label in ((e, "Euclidean"), (b, "Bregman")):
                    d = derive_diagnostics(run, _scenario_from_run(run)); ax.plot(run["t"], d[key], label=label)
                ax.set(title=f"Euclidean vs Bregman {name} ({form}, Post-Payload-Drop)", xlabel="time (s)", ylabel=name); ax.grid(True, linestyle=":"); ax.legend(); out.append(_save(fig, root / "euclidean_v_bregman" / f"{name}_{form}.png", visible))
    successful = {k: v for k, v in runs.items() if v.get("metrics")}
    for metric, title, ylabel in (("positionRMSE", "Position RMSE", "RMSE (m)"), ("attitudeRMSE", "Attitude RMSE", "RMSE (rad)"), ("wrenchRMS", "Control-wrench RMS", "RMS wrench")):
        if not successful: continue
        fig, ax = plt.subplots(figsize=(10, 5)); labels = list(successful); values = [float(successful[k]["metrics"][metric]) for k in labels]; ax.bar(labels, values); ax.set(title=title, ylabel=ylabel); ax.tick_params(axis="x", rotation=35); ax.grid(True, axis="y", linestyle=":"); out.append(_save(fig, suite / "comparisons" / "performance" / f"performance_{metric}.png", visible))
    return out
