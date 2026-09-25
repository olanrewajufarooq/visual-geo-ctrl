"""Unit-aware PDF/PNG exports grouped by paper subsection."""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from ..math.se3 import inv_se3, adjoint_se3
from ..sim.publication import NAMES, ADAPTIVE_MODES
from ..sim.paper_metrics import _pose_errors

STYLES = {"nominal": {"color": "#777777", "linestyle": ":"},
          "euclidean": {"color": "#D55E00", "linestyle": "-."},
          "bregman": {"color": "#0072B2", "linestyle": "-"}}
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
        ax.set_ylabel(label); ax.set_xlim(*xlim); ax.grid(True, alpha=.22, linewidth=.5)
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
    runs = {mode: runs[mode] for mode in ADAPTIVE_MODES if mode in runs}
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
                            label="Desired, transported per actual body" if mode == next(iter(runs)) else "_nolegend_")
        if kind in ("position", "attitude"):
            reference = first["Hdesired"][:, :3, 3] if kind == "position" else np.degrees(np.unwrap(Rotation.from_matrix(first["Hdesired"][:, :3, :3]).as_euler("xyz"), axis=0))
            for ax, j in zip(axes, indices): ax.plot(first["t"], reference[:, j], "k--", label="Desired reference")
        legend(fig, axes[0]); save(fig, out, "tracking_"+kind)

    fig, axes = panels([r"$\|p-p_d\|$ [m]", "Geodesic attitude\nerror [deg]", r"$\|s\|_{\Lambda_s}$"+"\n[metric units]"])
    for mode, run in runs.items():
        pos, angle = _pose_errors(run)
        metric = np.linalg.inv(scenarios[mode]["controller"]["Lambda"])
        r = np.sqrt(np.einsum("ni,ij,nj->n", run["s"], metric, run["s"]))
        for ax, values in zip(axes, (pos, np.degrees(angle), r)):
            ax.plot(run["t"], values, label=NAMES[mode], **STYLES[mode])
    legend(fig, axes[0]); save(fig, out, "adaptive_tracking_errors")

    fig, axes = panels([r"$\|f_c\|$ [N]", r"$\|\tau_c\|$ [N m]"])
    for mode, run in runs.items():
        for ax, sl in zip(axes, (slice(3, 6), slice(0, 3))):
            ax.plot(run["t"], np.linalg.norm(run["wrench"][:, sl], axis=1), label=NAMES[mode], **STYLES[mode])
    legend(fig, axes[0]); save(fig, out, "control_wrench_demand")

    fig, axes = panels([r"$\hat m$ [kg]", r"$\lambda_{\min}(\hat{\mathcal{J}})$"+"\n[SI coordinates]"])
    for mode, run in runs.items():
        if mode == "nominal": continue
        axes[0].plot(run["t"], run["estimatePi"][:, 0], label=NAMES[mode], **STYLES[mode])
        axes[1].plot(run["t"], run["minPseudoEigenvalue"], **STYLES[mode])
    axes[0].step(first["t"], first["activePlantPi"][:, 0], where="post", color="black", ls="--", label="True mass")
    axes[1].axhline(0, color="black", lw=.6)
    legend(fig, axes[0]); save(fig, Path(output_dir)/"02-physical-consistency", "physical_consistency")

    if "bregman" in runs:
        run = runs["bregman"]
        fig = plt.figure(figsize=(3.5, 3.3), layout="constrained")
        ax = fig.add_subplot(projection="3d")
        ax.plot(*run["Hdesired"][:, :3, 3].T, "k--", label="Desired reference")
        ax.plot(*run["H"][:, :3, 3].T, **STYLES["bregman"], label=NAMES["bregman"])
        i = np.searchsorted(run["t"], 10)
        if i < len(run["t"]): ax.scatter(*run["H"][i, :3, 3], marker="x", color="black", label="Release (10 s)")
        ax.set(xlabel="x [m]", ylabel="y [m]", zlabel="z [m]")
        ax.legend(loc="upper center", bbox_to_anchor=(.5, 1.15), fontsize=6)
        save(fig, out, "tracking_3d_bregman")


def theory_figures(run, scenario, root, summary, connection):
    out = Path(root)/"04-nominal-validation"
    # Do not leave a stale successful figure beside a newly failed check.
    obsolete = ["nominal_reaching_FAILED" if summary["passed"] else "nominal_reaching"]
    if not connection["passed"]: obsolete.append("connection_equivalence")
    for name in obsolete:
        for extension in ("pdf", "png"):
            (out/f"{name}.{extension}").unlink(missing_ok=True)
    metric = np.linalg.inv(scenario["controller"]["Lambda"])
    r = np.sqrt(np.einsum("ni,ij,nj->n", run["s"], metric, run["s"]))
    horizon = min(run["t"][-1], max(1., 1.6*(summary["T_obs"] or min(summary["T_bound"], 5.))))
    fig, axes = panels([r"$\|s\|_{\Lambda_s}$ [metric units]", "$V_s$ [J]"], False, (0, horizon))
    axes[0].plot(run["t"], r, color="#0072B2", label="Oracle known-inertia controller")
    axes[1].plot(run["t"], run["Vs"], color="#0072B2")
    axes[0].axhline(summary["epsilon_s"], color="gray", ls=":", label=r"$\epsilon_s$")
    for ax in axes:
        if summary["T_bound"] <= horizon:
            ax.axvline(summary["T_bound"], color="#D55E00", ls="--", label=r"$T_{\rm bound}$")
        else:
            ax.annotate(f"$T_{{bound}}$ = {summary['T_bound']:.3g} s (outside view)",
                        xy=(.995, .75), xytext=(.48, .75), xycoords="axes fraction",
                        textcoords="axes fraction", color="#D55E00", fontsize=7,
                        arrowprops={"arrowstyle": "->", "color": "#D55E00"})
        if summary["T_obs"] is not None: ax.axvline(summary["T_obs"], color="black", ls=":", label=r"$T_{\rm obs}$")
    legend(fig, axes[0]); save(fig, out, "nominal_reaching" if summary["passed"] else "nominal_reaching_FAILED")
    if connection["passed"]:
        fig, axes = panels(["Scaled wrench\nnorm", "Scaled residual\nnorm"], False, (0, horizon))
        axes[0].plot(connection["t"], connection["difference"], label=r"$\|\mathcal{W}_c^{RB}-\mathcal{W}_c^{LC}\|_*$")
        axes[0].plot(connection["t"], connection["theory"], "--", label=r"$\|K_{RB}(\mathcal{V})s\|_*$")
        axes[1].semilogy(connection["t"], np.maximum(connection["residual"], 1e-18))
        axes[1].set_xlabel(r"Time since initialization, $t$ [s]")
        if summary["T_obs"] is not None:
            for ax in axes:
                ax.axvline(summary["T_obs"], color="black", ls=":", label=r"$T_{\mathrm{obs}}$")
        legend(fig, axes[0]); save(fig, out, "connection_equivalence")
