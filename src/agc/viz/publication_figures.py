"""Unit-aware PDF/PNG exports grouped by paper subsection."""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from ..math.se3 import inv_se3, adjoint_se3
from ..math.inertia import center_and_principal_moments
from ..sim.publication import NAMES, PRIMARY_MODES
from ..sim.paper_metrics import _pose_errors

STYLES = {"nominal": {"color": "#FF0000", "linestyle": "--"},
          "euclidean": {"color": "#00FF00", "linestyle": "-."},
          "bregman": {"color": "#0000FF", "linestyle": "-"}}
CONNECTION_STYLES = {"lc": {"color": "#0000FF", "linestyle": "-"},
                     "rb": {"color": "#FF0000", "linestyle": "--"}}
plt.rcParams.update({"font.size": 8, "axes.labelsize": 9, "legend.fontsize": 7,
                     "lines.linewidth": 1.1, "pdf.fonttype": 42, "savefig.dpi": 400})


def save(fig, out, name):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    for extension in ("pdf", "png"):
        fig.savefig(out / f"{name}.{extension}", bbox_inches="tight")
    plt.close(fig)


def panels(labels, release=True, xlim=(0, 30)):
    fig, axes = plt.subplots(len(labels), 1, figsize=(7.16, 1.45*len(labels)+.55), sharex=True, layout="constrained")
    axes = np.atleast_1d(axes)
    for ax, label in zip(axes, labels):
        ax.set_ylabel(label); ax.set_xlim(*xlim); ax.margins(x=0); ax.grid(True, alpha=.22, linewidth=.5)
        if release: ax.axvline(10, color="black", ls=":", lw=.8)
    axes[-1].set_xlabel("Time [s]")
    return fig, axes


def legend(fig, ax):
    handles, labels = ax.get_legend_handles_labels()
    fig.legend(handles, labels, loc="outside upper center", ncol=2, frameon=False)


def transported_reference(run):
    return np.array([adjoint_se3(inv_se3(inv_se3(hd) @ h)) @ v
                     for h, hd, v in zip(run["H"], run["Hdesired"], run["Vdesired"])])


def adaptive_figures(runs, scenarios, output_dir):
    runs = {mode: runs[mode] for mode in PRIMARY_MODES if mode in runs}
    out = Path(output_dir) / "01-adaptive-tracking"
    first = next(iter(runs.values()))
    for kind, labels, indices in (
        ("position", [f"${c}$ [m]" for c in "xyz"], [0, 1, 2]),
        ("attitude", ["Roll [deg]", "Pitch [deg]", "Yaw [deg]"], [0, 1, 2]),
        ("linear_velocity", [f"$v_{c}$ [m/s]" for c in "xyz"], [3, 4, 5]),
        ("angular_velocity", [rf"$\omega_{c}$ [rad/s]" for c in "xyz"], [0, 1, 2]),
    ):
        fig, axes = panels(labels)
        for mode, run in runs.items():
            if kind == "position": actual = run["H"][:, :3, 3]
            elif kind == "attitude":
                actual = np.unwrap(Rotation.from_matrix(run["H"][:, :3, :3]).as_euler("xyz"), axis=0)
                reference = np.unwrap(Rotation.from_matrix(run["Hdesired"][:, :3, :3]).as_euler("xyz"), axis=0)
                actual += 2*np.pi*np.round((reference[0]-actual[0])/(2*np.pi))
                actual = np.degrees(actual)
            else: actual = run["V"]
            for ax, j in zip(axes, indices): ax.plot(run["t"], actual[:, j], label=NAMES[mode], **STYLES[mode])
            if "velocity" in kind:
                reference = transported_reference(run)
                for ax, j in zip(axes, indices):
                    ax.plot(run["t"], reference[:, j], "k--", lw=.7, alpha=.65,
                            label="Transported reference" if mode == next(iter(runs)) else "_nolegend_")
        if kind in ("position", "attitude"):
            reference = first["Hdesired"][:, :3, 3] if kind == "position" else np.degrees(np.unwrap(Rotation.from_matrix(first["Hdesired"][:, :3, :3]).as_euler("xyz"), axis=0))
            for ax, j in zip(axes, indices): ax.plot(first["t"], reference[:, j], "k--", label="Desired reference")
        legend(fig, axes[0]); save(fig, out, "tracking_"+kind)

    fig, axes = panels([r"$\|p-p_d\|$ [m]", "Geodesic attitude\nerror [deg]", r"$\|s\|_{\Lambda_s}$"])
    for mode, run in runs.items():
        pos, angle = _pose_errors(run)
        metric = np.asarray(scenarios[mode]["controller"].get("Lambda_s", np.linalg.inv(scenarios[mode]["controller"]["Lambda"])))
        r = np.sqrt(np.einsum("ni,ij,nj->n", run["s"], metric, run["s"]))
        for ax, values in zip(axes, (pos, np.degrees(angle), r)):
            ax.plot(run["t"], values, label=NAMES[mode], **STYLES[mode])
    legend(fig, axes[0]); save(fig, out, "adaptive_tracking_errors")

    fig, axes = panels([r"$\|f_c\|$ [N]", r"$\|\tau_c\|$ [N m]"])
    for mode, run in runs.items():
        for ax, sl in zip(axes, (slice(3, 6), slice(0, 3))):
            ax.plot(run["t"], np.linalg.norm(run["wrench"][:, sl], axis=1), label=NAMES[mode], **STYLES[mode])
    legend(fig, axes[0]); save(fig, out, "control_wrench_demand")

    fig, axes = panels([r"$\hat m$ [kg]", r"$\lambda_{\min}(\hat{\mathcal{J}})$"])
    for mode, run in runs.items():
        if mode == "nominal": continue
        axes[0].plot(run["t"], run["estimatePi"][:, 0], label=NAMES[mode], **STYLES[mode])
        axes[1].plot(run["t"], run["minPseudoEigenvalue"], **STYLES[mode])
    axes[0].step(first["t"], first["activePlantPi"][:, 0], where="post", color="black", ls="--", label="True mass")
    axes[1].axhline(0, color="black", lw=.6)
    legend(fig, axes[0]); save(fig, Path(output_dir)/"02-physical-consistency", "physical_consistency")
    inertial_estimate_figures(runs, output_dir)

    if "bregman" in runs:
        run = runs["bregman"]
        fig = plt.figure(figsize=(3.5, 3.3), layout="constrained")
        ax = fig.add_subplot(projection="3d")
        ax.plot(*run["Hdesired"][:, :3, 3].T, "k--", label="Desired reference")
        ax.plot(*run["H"][:, :3, 3].T, **STYLES["bregman"], label=NAMES["bregman"])
        i = np.searchsorted(run["t"], 10)
        if i < len(run["t"]): ax.scatter(*run["H"][i, :3, 3], marker="x", color="black", label="Release (10 s)")
        ax.set(xlabel="x [m]", ylabel="y [m]", zlabel="z [m]")
        fig.legend(*ax.get_legend_handles_labels(), loc="outside upper center", ncol=2,
                   frameon=False, fontsize=6)
        save(fig, out, "tracking_3d_bregman")


def inertial_estimate_figures(runs, output_dir):
    """Export additional estimator diagnostics without rerunning the plant."""
    adaptive = {mode: runs[mode] for mode in ("euclidean", "bregman") if mode in runs}
    if not adaptive:
        return
    first = next(iter(adaptive.values()))
    truth = center_and_principal_moments(first["activePlantPi"])
    estimates = {mode: center_and_principal_moments(run["estimatePi"])
                 for mode, run in adaptive.items()}
    for index, name, labels in (
        (0, "estimated_center_of_mass", [rf"$\hat c_{{{axis}}}$ [m]" for axis in "xyz"]),
        (1, "estimated_principal_inertia", [rf"$\hat J_{{c,{i}}}$ [kg m$^2$]" for i in (1, 2, 3)]),
    ):
        fig, axes = panels(labels)
        for mode, run in adaptive.items():
            for j, ax in enumerate(axes):
                ax.plot(run["t"], estimates[mode][index][:, j], label=NAMES[mode], **STYLES[mode])
        for j, ax in enumerate(axes):
            ax.step(first["t"], truth[index][:, j], where="post", color="black",
                    ls="--", label="True value")
        legend(fig, axes[0])
        save(fig, Path(output_dir)/"02-physical-consistency", name)


def connection_realization_figures(runs, scenarios, output_dir):
    """Closed-loop LC/RB comparison; runs are independently simulated but matched."""
    out = Path(output_dir) / "04-nominal-validation"
    first = runs["lc"]
    time_offset = float(scenarios["lc"].get("timeOffset", 0.0))
    display_end = float(scenarios["lc"].get("displayEnd", time_offset + first["t"][-1]))
    xlim = (time_offset, display_end)
    for kind, labels, indices in (
        ("position", [f"${c}$ [m]" for c in "xyz"], [0, 1, 2]),
        ("attitude", ["Roll [deg]", "Pitch [deg]", "Yaw [deg]"], [0, 1, 2]),
        ("linear_velocity", [f"$v_{c}$ [m/s]" for c in "xyz"], [3, 4, 5]),
        ("angular_velocity", [rf"$\omega_{c}$ [rad/s]" for c in "xyz"], [0, 1, 2]),
    ):
        fig, axes = panels(labels, release=False, xlim=xlim)
        axes[-1].set_xlabel("Lemniscate time [s]" if time_offset else "Time [s]")
        first_visible = first["t"] + time_offset <= display_end
        if kind in ("position", "attitude"):
            reference = first["Hdesired"][:, :3, 3] if kind == "position" else np.degrees(np.unwrap(Rotation.from_matrix(first["Hdesired"][:, :3, :3]).as_euler("xyz"), axis=0))
            for ax, j in zip(axes, indices):
                ax.plot(time_offset + first["t"][first_visible], reference[first_visible, j], "k--", lw=.7, alpha=.65, label="Desired reference")
        elif "velocity" in kind:
            for key, reference_run in runs.items():
                reference = transported_reference(reference_run)
                reference_visible = reference_run["t"] + time_offset <= display_end
                for ax, j in zip(axes, indices):
                    ax.plot(time_offset + reference_run["t"][reference_visible], reference[reference_visible, j], "k--", lw=.7, alpha=.65,
                            label="Transported reference" if key == "lc" else "_nolegend_")
        for key, run in runs.items():
            visible = run["t"] + time_offset <= display_end
            if kind == "position": actual = run["H"][:, :3, 3]
            elif kind == "attitude":
                actual = np.unwrap(Rotation.from_matrix(run["H"][:, :3, :3]).as_euler("xyz"), axis=0)
                reference = np.unwrap(Rotation.from_matrix(run["Hdesired"][:, :3, :3]).as_euler("xyz"), axis=0)
                actual += 2*np.pi*np.round((reference[0]-actual[0])/(2*np.pi))
                actual = np.degrees(actual)
            else: actual = run["V"]
            for ax, j in zip(axes, indices): ax.plot(time_offset + run["t"][visible], actual[visible, j], label=rf"$C_{{\mathrm{{{key.upper()}}}}}$", **CONNECTION_STYLES[key])
        legend(fig, axes[0]); save(fig, out, "connection_tracking_"+kind)

    fig, axes = panels([r"$\|p-p_d\|$ [m]", "Geodesic attitude\nerror [deg]", r"$\|s\|_{\Lambda_s}$"], release=False, xlim=xlim)
    axes[-1].set_xlabel("Lemniscate time [s]" if time_offset else "Time [s]")
    for key, run in runs.items():
        visible = run["t"] + time_offset <= display_end
        pos, angle = _pose_errors(run)
        metric = np.asarray(scenarios[key]["controller"].get("Lambda_s", np.linalg.inv(scenarios[key]["controller"]["Lambda"])))
        weighted = np.sqrt(np.einsum("ni,ij,nj->n", run["s"], metric, run["s"]))
        for ax, values in zip(axes, (pos, np.degrees(angle), weighted)):
            ax.plot(time_offset + run["t"][visible], values[visible], label=rf"$C_{{\mathrm{{{key.upper()}}}}}$", **CONNECTION_STYLES[key])
    legend(fig, axes[0]); save(fig, out, "connection_realization_comparison")


def theory_figures(run, scenario, root, summary, connection):
    out = Path(root)/"04-nominal-validation"
    # Do not leave a stale successful figure beside a newly failed check.
    obsolete = ["nominal_reaching_FAILED" if summary["passed"] else "nominal_reaching"]
    if not connection["passed"]: obsolete.append("connection_equivalence")
    for name in obsolete:
        for extension in ("pdf", "png"):
            (out/f"{name}.{extension}").unlink(missing_ok=True)
    metric = np.asarray(scenario["controller"].get("Lambda_s", np.linalg.inv(scenario["controller"]["Lambda"])))
    r = np.sqrt(np.einsum("ni,ij,nj->n", run["s"], metric, run["s"]))
    time_offset = float(scenario.get("timeOffset", 0.0))
    horizon = float(summary.get("final_source_time_s", scenario.get("displayEnd", 30.0)))
    visible_run = time_offset + np.asarray(run["t"]) <= horizon
    run_time = time_offset + np.asarray(run["t"])[visible_run]
    fig, axes = panels([r"$\|s\|_{\Lambda_s}$", "$V_s$ [J]"], False, (time_offset, horizon))
    axes[0].plot(run_time, r[visible_run], color="#FF0000", label="Known-inertia controller")
    axes[1].plot(run_time, np.asarray(run["Vs"])[visible_run], color="#FF0000")
    axes[0].set_yscale("symlog", linthresh=summary["epsilon_s"])
    axes[0].axhline(summary["epsilon_s"], color="black", ls=":", label=r"$\epsilon_s$")
    for index, ax in enumerate(axes):
        if time_offset + summary["T_bound"] <= horizon:
            ax.axvline(time_offset + summary["T_bound"], color="#FF0000", ls="--", label=r"$T_{\rm bound}$")
        if summary["T_obs"] is not None:
            ax.axvline(time_offset + summary["T_obs"], color="black", ls=":", label=r"$T_{\mathrm{obs}}$")
    if time_offset + summary["T_bound"] > horizon:
        axes[0].text(.99, .96,
                     rf"$T_{{\mathrm{{bound}}}}={summary['T_bound']:.2f}\,\mathrm{{s}}$ elapsed (outside view)",
                     transform=axes[0].transAxes, ha="right", va="top", color="#FF0000", fontsize=7)
    axes[-1].set_xlabel("Lemniscate time [s]" if time_offset else "Time [s]")
    legend(fig, axes[0]); save(fig, out, "nominal_reaching" if summary["passed"] else "nominal_reaching_FAILED")
    if connection["passed"]:
        fig, axes = panels(["Scaled wrench\nnorm", "Scaled residual\nnorm"], False,
                            (time_offset, horizon))
        visible = time_offset + np.asarray(connection["t"]) <= horizon
        time = time_offset + np.asarray(connection["t"])[visible]
        axes[0].plot(time, np.asarray(connection["difference"])[visible], color="black",
                     label=r"$\|\mathcal{W}_c^{RB}-\mathcal{W}_c^{LC}\|_*$")
        axes[1].semilogy(time, np.maximum(np.asarray(connection["residual"])[visible], 1e-18), color="black")
        axes[1].set_xlabel("Lemniscate time [s]" if time_offset else r"Time since initialization, $t$ [s]")
        if summary["T_obs"] is not None:
            for ax in axes:
                ax.axvline(time_offset + summary["T_obs"], color="black", ls=":", label=r"$T_{\mathrm{obs}}$")
        legend(fig, axes[0]); save(fig, out, "connection_equivalence")
