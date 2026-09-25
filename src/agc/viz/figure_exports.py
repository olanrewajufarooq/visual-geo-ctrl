"""PNG-only standalone and suite figure exports matching MATLAB coverage."""

from pathlib import Path
from typing import Any, Dict, List, Optional
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from .diagnostics import derive_diagnostics, slice_release_window
from .plot_style import COMPONENT_COLORS, color_for_series, get_time_horizon, limit_for, plot_time_series, wrap_degrees
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


def _time_plot(t, series, labels, title, ylabel, family, path, horizon, release, failure_time, visible):
    fig, ax = plt.subplots(figsize=(9, 5))
    plot_time_series(ax, t, series, labels, family, horizon, failure_time=failure_time, release=release)
    ax.set(title=title, xlabel="time (s)", ylabel=ylabel); ax.grid(True, linestyle=":"); ax.legend(loc="best")
    return _save(fig, path, visible)


def _render_run(run, scenario, output_dir: Path, visible=False, suffix="", horizon=None, release=None, failure_time=None) -> List[Dict[str, Any]]:
    d = derive_diagnostics(run, scenario)
    t = np.asarray(run["t"], dtype=float); rel = release
    if horizon is None:
        horizon = get_time_horizon(run)
    out = []
    def tp(name, series, labels, title, ylabel, family, t_use=None):
        t_plot = t if t_use is None else t_use
        rec = _time_plot(t_plot, series, labels, title + suffix, ylabel, family, output_dir / f"{name}.png", horizon, rel, failure_time, visible)
        out.append(rec)
    # The simulation logs at 500 Hz for numerical fidelity.  Plotting every sample at 300 DPI
    # produces sub-pixel strokes that merge into solid painted blobs.  Instead, keep at most
    # PLOT_MAX_POINTS evenly-spaced samples: stride = len(t) // PLOT_MAX_POINTS.  The values
    # plotted are the true logged values at those instants — no smoothing, no interpolation.
    PLOT_MAX_POINTS = 500
    s = max(1, len(t) // PLOT_MAX_POINTS)
    t_ds = t[::s]
    V_ds = np.asarray(run["V"])[::s]
    Vd_ds = np.asarray(run["Vdesired"])[::s]
    wrench_ds = np.asarray(run["wrench"])[::s]
    pos_ds = d["position"][::s]; desPos_ds = d["desiredPosition"][::s]
    rpy_ds = d["rpy"][::s]; desRpy_ds = d["desiredRpy"][::s]
    sNorm_ds = d["slidingNorm"][::s]
    Psi_ds = np.asarray(run["Psi"])[::s]
    Vs_ds = d["Vs"][::s]
    # Trajectory views (keep full resolution — no time axis, so no paint issue)
    fig = plt.figure(figsize=(8, 6)); ax = fig.add_subplot(111, projection="3d")
    ax.plot(d["desiredPosition"][:, 0], d["desiredPosition"][:, 1], d["desiredPosition"][:, 2], color=COMPONENT_COLORS[0], linestyle="--", label="desired")
    ax.plot(d["position"][:, 0], d["position"][:, 1], d["position"][:, 2], color=COMPONENT_COLORS[0], label="actual")
    ax.set_xlim(*limit_for("trajectory_x")); ax.set_ylim(*limit_for("trajectory_y")); ax.set_zlim(*limit_for("trajectory_z"))
    ax.set(title="Trajectory" + suffix, xlabel="x (m)", ylabel="y (m)", zlabel="z (m)"); ax.grid(True); ax.legend()
    out.append(_save(fig, output_dir / "trajectory_3d.png", visible))
    fig, ax = plt.subplots(figsize=(8, 6)); ax.plot(d["desiredPosition"][:, 0], d["desiredPosition"][:, 1], color=COMPONENT_COLORS[0], linestyle="--", label="desired"); ax.plot(d["position"][:, 0], d["position"][:, 1], color=COMPONENT_COLORS[0], label="actual"); ax.set(title="XY trajectory" + suffix, xlabel="x (m)", ylabel="y (m)"); ax.set_xlim(*limit_for("trajectory_x")); ax.set_ylim(*limit_for("trajectory_y")); ax.grid(True); ax.legend(); out.append(_save(fig, output_dir / "trajectory_xy.png", visible))
    tp("altitude", [pos_ds[:, 2], desPos_ds[:, 2]], ["actual", "desired"], "Altitude", "z (m)", "altitude", t_ds)
    tp("position", [pos_ds[:, i] for i in range(3)] + [desPos_ds[:, i] for i in range(3)], ["x", "y", "z", "x desired", "y desired", "z desired"], "Cartesian position", "p (m)", "position", t_ds)
    tp("attitude", [wrap_degrees(np.degrees(rpy_ds[:, i])) for i in range(3)] + [wrap_degrees(np.degrees(desRpy_ds[:, i])) for i in range(3)], ["roll", "pitch", "yaw", "roll desired", "pitch desired", "yaw desired"], "Attitude", "angle (deg)", "orientation", t_ds)
    tp("linear_velocity", [V_ds[:, i] for i in range(3, 6)] + [Vd_ds[:, i] for i in range(3, 6)], ["v_x", "v_y", "v_z", "v_x desired", "v_y desired", "v_z desired"], "Body linear velocity", "v (m/s)", "linear_velocity", t_ds)
    tp("angular_velocity", [V_ds[:, i] for i in range(3)] + [Vd_ds[:, i] for i in range(3)], ["omega_x", "omega_y", "omega_z", "omega_x desired", "omega_y desired", "omega_z desired"], "Body angular velocity", "omega (rad/s)", "angular_velocity", t_ds)
    tp("wrench_force", [wrench_ds[:, i] for i in range(3, 6)], ["F_x", "F_y", "F_z"], "Control force", "force (N)", "force", t_ds)
    tp("wrench_torque", [wrench_ds[:, i] for i in range(3)], ["tau_x", "tau_y", "tau_z"], "Control torque", "torque (Nm)", "torque", t_ds)
    tp("sliding_norm", [sNorm_ds], ["||s||"], "Sliding residual", "||s||", "sliding_norm", t_ds)
    tp("potential", [Psi_ds], ["Psi"], "Configuration potential", "Psi", "potential", t_ds)
    tp("transverse_energy", [Vs_ds], ["V_s"], "Transverse energy", "V_s", "transverse_energy", t_ds)
    if str(run.get("mode", "")).lower() != "nominal":
        tp("parameter_error", [d["parameterError"][::s]], ["normalized parameter error"], "Parameter error", "normalized error", "parameter_error", t_ds)
        tp("pseudo_inertia_margin", [d["pseudoMargin"][::s]], ["min eig(Jhat)"], "Pseudo-inertia margin", "minimum eigenvalue", "pseudo_margin", t_ds)
        tp("mass", [d["estimatePi"][::s, 0], d["truePi"][::s, 0]], ["estimate", "true"], "Mass estimate", "mass (kg)", "mass", t_ds)
        tp("cog", [d["cog"][::s, i] for i in range(3)] + [d["trueCog"][::s, i] for i in range(3)], ["rx", "ry", "rz", "rx true", "ry true", "rz true"], "Center of mass", "CoG (m)", "cog", t_ds)
    tp("inertia_principal", [d["estimatePi"][::s, i] for i in range(4, 7)] + [d["truePi"][::s, i] for i in range(4, 7)], ["Ixx", "Iyy", "Izz", "Ixx true", "Iyy true", "Izz true"], "Principal inertia", "inertia (kg m^2)", "inertia_principal", t_ds)
    tp("inertia_off_diagonal", [d["estimatePi"][::s, i] for i in range(7, 10)] + [d["truePi"][::s, i] for i in range(7, 10)], ["Ixy", "Ixz", "Iyz", "Ixy true", "Ixz true", "Iyz true"], "Off-diagonal inertia", "inertia (kg m^2)", "inertia_off_diagonal", t_ds)
    return out


def export_run_figures(run: Dict[str, Any], output_dir: str, include_relative: bool = True, visible: bool = False) -> List[Dict[str, Any]]:
    scenario = _scenario_from_run(run)
    output = Path(output_dir)
    failure = run.get("failure") or {}
    failure_time = float(failure["time"]) if failure.get("time") is not None else None
    release = _release_time(run)
    records = _render_run(run, scenario, output / "figures" / "total-sim", visible, " (Total Simulation)", get_time_horizon(run), release, failure_time)
    _, total_horizon = get_time_horizon(run)
    if include_relative and release is not None and release < total_horizon and release < float(run["t"][-1]):
        diagnostics = derive_diagnostics(run, scenario)
        run_drop, _ = slice_release_window(run, diagnostics, release)
        drop_failure = failure_time - release if failure_time is not None else None
        records.extend(_render_run(run_drop, scenario, output / "figures" / "from-drop", visible, " (Post-Payload-Drop)", get_time_horizon(run, release), None, drop_failure))
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
            pairs = [window_runs.get(f"{ctrl}_lc"), window_runs.get(f"{ctrl}_rb")]
            if not all(pairs): continue
            comparison_horizon = (0.0, max(get_time_horizon(run)[1] for run in pairs))
            fig3 = plt.figure(figsize=(8, 6)); ax3 = fig3.add_subplot(111, projection="3d")
            for run, label in zip(pairs, (r"$C_{\mathrm{LC}}$", r"$C_{\mathrm{RB}}$")):
                d = derive_diagnostics(run, _scenario_from_run(run)); ax3.plot(d["position"][:, 0], d["position"][:, 1], d["position"][:, 2], color=color_for_series(label), linestyle="--" if "RB" in label else "-", label=label)
            ax3.set_xlim(*limit_for("trajectory_x")); ax3.set_ylim(*limit_for("trajectory_y")); ax3.set_zlim(*limit_for("trajectory_z"))
            ax3.set(title=ctrl + r" $C_{\mathrm{LC}} / C_{\mathrm{RB}}$ trajectory", xlabel="x (m)", ylabel="y (m)", zlabel="z (m)"); ax3.legend(); ax3.grid(True); out.append(_save(fig3, suite / "comparisons" / "total-sim" / f"{ctrl}_trajectory_lc_rb.png", visible))
            fig, axes = plt.subplots(3, 2, figsize=(12, 9), sharex=True)
            for row, key in enumerate(("positionError", "attitudeError", "slidingNorm")):
                for run, label in zip(pairs, (r"$C_{\mathrm{LC}}$", r"$C_{\mathrm{RB}}$")):
                    sc = _scenario_from_run(run); d = derive_diagnostics(run, sc)
                    values = np.degrees(d[key]) if key == "attitudeError" else d[key]
                    family = {"positionError": "position_error", "attitudeError": "attitude_error", "slidingNorm": "sliding_norm"}[key]
                    plot_time_series(axes[row, 0], run["t"], [values], [label], family, comparison_horizon)
                axes[row, 0].set_title(f"{ctrl} {key}"); axes[row, 0].legend(); axes[row, 0].grid(True, linestyle=":")
            # Add energy, wrench, and trajectory comparisons as separate MATLAB-equivalent claims.
            for name, key, ylabel in (("transverse_energy", "Vs", "V_s"), ("wrench_norm", "wrenchNorm", "||W||")):
                fig2, ax2 = plt.subplots(figsize=(9, 5))
                for run, label in zip(pairs, (r"$C_{\mathrm{LC}}$", r"$C_{\mathrm{RB}}$")):
                    sc = _scenario_from_run(run); d = derive_diagnostics(run, sc); vals = d[key] if key in d else d["wrenchNorm"]
                    family = "transverse_energy" if name == "transverse_energy" else "wrench_norm"
                    plot_time_series(ax2, run["t"], [vals], [label], family, comparison_horizon)
                ax2.set(title=f"{ctrl} {name}", xlabel="time (s)", ylabel=ylabel); ax2.grid(True, linestyle=":"); ax2.legend(); out.append(_save(fig2, root / f"{ctrl}_{name}.png", visible))
            out.append(_save(fig, root / f"{ctrl}_lc_rb_comparison.png", visible))
    for form in ("lc", "rb"):
        e, b = runs.get(f"euclidean_{form}"), runs.get(f"bregman_{form}")
        if e is None or b is None: continue
        fields = (("position_error", "positionError", "Position error"), ("attitude_error", "attitudeError", "Attitude error"), ("sliding_norm", "slidingNorm", "Sliding norm"), ("parameter_error", "parameterError", "Parameter error"), ("pseudo_inertia_margin", "pseudoMargin", "Pseudo-inertia margin"))
        for name, key, title in fields:
            fig, ax = plt.subplots(figsize=(9, 5))
            comparison_horizon = (0.0, max(get_time_horizon(run)[1] for run in (e, b)))
            for run, label in ((e, "Euclidean"), (b, "Bregman")):
                d = derive_diagnostics(run, _scenario_from_run(run)); values = np.degrees(d[key]) if key == "attitudeError" else d[key]
                family = {"positionError": "position_error", "attitudeError": "attitude_error", "slidingNorm": "sliding_norm", "parameterError": "parameter_error", "pseudoMargin": "pseudo_margin"}[key]
                plot_time_series(ax, run["t"], [values], [label], family, comparison_horizon)
            ax.set(title=f"Euclidean vs Bregman {title.lower()} ({form})", xlabel="time (s)", ylabel=title); ax.grid(True, linestyle=":"); ax.legend(); out.append(_save(fig, suite / "comparisons" / "euclidean_v_bregman" / f"{name}_{form}.png", visible))

    # Repeat the unit-consistent comparisons in a release-relative window.
    releases = [_release_time(run) for run in runs.values()]
    release = next((value for value in releases if value is not None), None)
    if release is not None:
        drop_runs = {}
        for key, run in runs.items():
            if release < get_time_horizon(run)[1] and release < float(run["t"][-1]):
                sc = _scenario_from_run(run); d = derive_diagnostics(run, sc); drop_runs[key], _ = slice_release_window(run, d, release)
        root = suite / "comparisons" / "from-drop"
        for ctrl in controllers:
            lc, rb = drop_runs.get(f"{ctrl}_lc"), drop_runs.get(f"{ctrl}_rb")
            if lc is None or rb is None: continue
            comparison_horizon = (0.0, max(get_time_horizon(run, release)[1] for run in (runs[f"{ctrl}_lc"], runs[f"{ctrl}_rb"])))
            for name, key in (("position_error", "positionError"), ("attitude_error", "attitudeError"), ("sliding_norm", "slidingNorm"), ("transverse_energy", "Vs"), ("wrench_norm", "wrenchNorm")):
                fig, ax = plt.subplots(figsize=(9, 5))
                for run, label in ((lc, r"$C_{\mathrm{LC}}$"), (rb, r"$C_{\mathrm{RB}}$")):
                    d = derive_diagnostics(run, _scenario_from_run(run)); values = np.degrees(d[key]) if key == "attitudeError" else d[key]
                    family = {"positionError": "position_error", "attitudeError": "attitude_error", "slidingNorm": "sliding_norm", "Vs": "transverse_energy", "wrenchNorm": "wrench_norm"}[key]
                    plot_time_series(ax, run["t"], [values], [label], family, comparison_horizon)
                ax.set(title=f"{ctrl} {name} (Post-Payload-Drop)", xlabel="time (s)", ylabel=name); ax.grid(True, linestyle=":"); ax.legend(); out.append(_save(fig, root / f"{ctrl}_{name}.png", visible))
        for form in ("lc", "rb"):
            e, b = drop_runs.get(f"euclidean_{form}"), drop_runs.get(f"bregman_{form}")
            if e is None or b is None: continue
            comparison_horizon = (0.0, max(get_time_horizon(run, release)[1] for run in (runs[f"euclidean_{form}"], runs[f"bregman_{form}"])))
            for name, key in (("position_error", "positionError"), ("attitude_error", "attitudeError"), ("sliding_norm", "slidingNorm"), ("parameter_error", "parameterError"), ("pseudo_inertia_margin", "pseudoMargin")):
                fig, ax = plt.subplots(figsize=(9, 5))
                for run, label in ((e, "Euclidean"), (b, "Bregman")):
                    d = derive_diagnostics(run, _scenario_from_run(run)); values = np.degrees(d[key]) if key == "attitudeError" else d[key]
                    family = {"positionError": "position_error", "attitudeError": "attitude_error", "slidingNorm": "sliding_norm", "parameterError": "parameter_error", "pseudoMargin": "pseudo_margin"}[key]
                    plot_time_series(ax, run["t"], [values], [label], family, comparison_horizon)
                ax.set(title=f"Euclidean vs Bregman {name} ({form}, Post-Payload-Drop)", xlabel="time (s)", ylabel=name); ax.grid(True, linestyle=":"); ax.legend(); out.append(_save(fig, root / "euclidean_v_bregman" / f"{name}_{form}.png", visible))
    successful = {k: v for k, v in runs.items() if v.get("metrics")}
    for metric, title, ylabel in (("positionRMSE", "Position RMSE", "RMSE (m)"), ("attitudeRMSE", "Attitude RMSE", "RMSE (rad)"), ("wrenchRMS", "Control-wrench RMS", "RMS wrench")):
        if not successful: continue
        fig, ax = plt.subplots(figsize=(10, 5)); labels = list(successful); values = [float(successful[k]["metrics"][metric]) for k in labels]; ax.bar(labels, values, color=COMPONENT_COLORS[0]); ax.set(title=title, ylabel=ylabel); ax.tick_params(axis="x", rotation=35); ax.grid(True, axis="y", linestyle=":"); out.append(_save(fig, suite / "comparisons" / "performance" / f"performance_{metric}.png", visible))
    return out
