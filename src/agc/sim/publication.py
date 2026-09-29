"""Publication protocol: shared gains, explicit units, and auditable validation.

No optimization is performed here. Saved adaptation gains are provisional.
"""
from copy import deepcopy
import csv
import json
from pathlib import Path
import numpy as np

from .default_scenario import default_scenario
from .paper_metrics import (_pose_errors, compute_recovery_time, compute_persistent_reaching_time,
                           compute_nominal_reaching_bound)
from ..math.inertia import inertia_from_pi
from ..config.optimized_gains import optimized_gains

NAMES = {"nominal": "Known-inertia controller", "euclidean": "Euclidean adaptive controller",
         "bregman": "Natural/Bregman adaptive controller"}
ADAPTIVE_MODES = ("euclidean", "bregman")
PRIMARY_MODES = ("nominal",) + ADAPTIVE_MODES
COMMON_KEYS = ("KR", "Kxi", "Lambda", "kd", "ks", "alpha", "gravity")


def paper_scenario(mode, duration=30.):
    scenario = default_scenario(mode=mode, duration=duration, coriolis="lc", enable_pacing=False)
    if mode.lower() == "nominal":
        # In the payload-release comparison the known-inertia controller is
        # the baseline for the adaptive controllers, so it shares their
        # adaptive-base tracking gains. The separate nominal validation
        # experiment uses nominal_scenario() and its own no-payload setup.
        gains = optimized_gains("adaptive_base", "lc")
        scenario["controller"].update(
            KR=np.diag(gains["KRdiag"]),
            Kxi=np.diag(gains["Kxidiag"]),
            Lambda=np.diag(gains["LambdaDiag"]),
            Lambda_s=np.diag(1.0 / np.asarray(gains["LambdaDiag"], dtype=float)),
            kd=float(gains["kd"]),
            ks=float(gains["ks"]),
            alpha=float(gains["alpha"]),
        )
        scenario["controller"]["knownInertiaSchedule"] = "active-plant"
    return scenario


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    def convert(x):
        if isinstance(x, np.ndarray): return x.tolist()
        if isinstance(x, np.generic): return x.item()
        raise TypeError(type(x).__name__)
    path.write_text(json.dumps(value, indent=2, default=convert, allow_nan=False) + "\n", encoding="utf-8")


def write_csv(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        for row in rows:
            writer.writerow({k: f"{v:.6g}" if isinstance(v, float) else v for k, v in row.items()})


def baseline_metrics(run, mode, failure=None):
    t = run["t"]
    position, attitude = _pose_errors(run)
    attitude = np.degrees(attitude)
    row = {"Controller": NAMES[mode], "Status": "failed / partial data" if failure else "completed"}
    for label, mask in (("Pre-release", t < 10), ("Post-release", t >= 10)):
        for name, values, unit in (("position", position, "m"), ("attitude", attitude, "deg")):
            row[f"{label} {name} RMSE [{unit}]"] = float(np.sqrt(np.mean(values[mask]**2))) if mask.any() else None
            if label == "Post-release":
                row[f"{label} maximum {name} error [{unit}]"] = float(np.max(values[mask])) if mask.any() else None
    recovery = compute_recovery_time(run, 10.) if len(t) else None
    row["Absolute recovery time [s]"] = recovery
    row["Recovery duration [s]"] = None if recovery is None else recovery - 10.
    for name, values, unit in (("force", run["wrench"][:, 3:], "N"), ("torque", run["wrench"][:, :3], "N m")):
        norms = np.linalg.norm(values, axis=1)
        row[f"RMS {name} [{unit}]"] = float(np.sqrt(np.mean(norms**2))) if len(t) else None
        row[f"Peak {name} [{unit}]"] = float(norms.max()) if len(t) else None
    margin = run["minPseudoEigenvalue"]
    row["Minimum pseudo-inertia eigenvalue [SI coordinates]"] = float(margin.min()) if len(t) and mode != "nominal" else None
    row["Minimum post-release eigenvalue [SI coordinates]"] = float(margin[t >= 10].min()) if (t >= 10).any() and mode != "nominal" else None
    row["Final estimated mass [kg]"] = float(run["estimatePi"][-1, 0]) if len(t) else None
    row["Nonpositive pseudo-inertia eigenvalue"] = bool((margin <= 0).any()) if mode != "nominal" else None
    return row


def adaptive_performance_row(run, mode, failure=None):
    """Paper table row with controller-facing names and separate effort units."""
    row = baseline_metrics(run, mode, failure)
    return {
        "Controller": row["Controller"],
        "Status": row["Status"],
        "Pre-release position RMSE [m]": row["Pre-release position RMSE [m]"],
        "Post-release position RMSE [m]": row["Post-release position RMSE [m]"],
        "Post-release maximum position error [m]": row["Post-release maximum position error [m]"],
        "Pre-release geodesic-attitude RMSE [deg]": row["Pre-release attitude RMSE [deg]"],
        "Post-release geodesic-attitude RMSE [deg]": row["Post-release attitude RMSE [deg]"],
        "Post-release maximum attitude error [deg]": row["Post-release maximum attitude error [deg]"],
        "Recovery duration [s]": row["Recovery duration [s]"],
        "RMS force norm [N]": row["RMS force [N]"],
        "Peak force norm [N]": row["Peak force [N]"],
        "RMS torque norm [N m]": row["RMS torque [N m]"],
        "Peak torque norm [N m]": row["Peak torque [N m]"],
        "Minimum lambda_min(Jhat)": row["Minimum pseudo-inertia eigenvalue [SI coordinates]"],
    }


def physical_consistency_row(run, mode):
    if mode not in ADAPTIVE_MODES:
        raise ValueError("Physical-consistency table is defined for adaptive estimators only")
    margin = np.asarray(run["minPseudoEigenvalue"], dtype=float)
    t = np.asarray(run["t"], dtype=float)
    post_release = margin[t >= 10.]
    return {
        "Controller": NAMES[mode],
        "Initial estimated mass [kg]": float(run["estimatePi"][0, 0]),
        "Final estimated mass [kg]": float(run["estimatePi"][-1, 0]),
        "Minimum lambda_min(Jhat)": float(np.min(margin)),
        "Minimum post-release lambda_min(Jhat)": float(np.min(post_release)) if len(post_release) else None,
        "Physical-consistency violation?": "yes" if bool(np.any(margin <= 0.)) else "no",
    }


def connection_realization_row(run, connection, scenario, epsilon=1e-8, final_time=30., time_offset=10.):
    position, attitude = _pose_errors(run)
    metric = np.asarray(scenario["controller"].get("Lambda_s", np.linalg.inv(scenario["controller"]["Lambda"])))
    weighted = np.sqrt(np.einsum("ni,ij,nj->n", run["s"], metric, run["s"]))
    force = np.linalg.norm(run["wrench"][:, 3:], axis=1)
    torque = np.linalg.norm(run["wrench"][:, :3], axis=1)
    return {
        "Connection realization": rf"C_{connection.upper()}",
        "Position RMSE [m]": float(np.sqrt(np.mean(position**2))),
        "Geodesic attitude RMSE [deg]": float(np.degrees(np.sqrt(np.mean(attitude**2)))),
        "Maximum ||s||_{Lambda_s}": float(np.max(weighted)),
        "T_obs [source s]": (None if (elapsed := compute_persistent_reaching_time(
            run, epsilon, metric, final_time=final_time, time_offset=time_offset)) is None
            else elapsed + time_offset),
        "RMS force norm [N]": float(np.sqrt(np.mean(force**2))),
        "Peak force norm [N]": float(np.max(force)),
        "RMS torque norm [N m]": float(np.sqrt(np.mean(torque**2))),
        "Peak torque norm [N m]": float(np.max(torque)),
    }


def controller_gain_rows(scenarios):
    """Machine-readable record of actual gains used in the saved runs."""
    nominal = scenarios["nominal"]["controller"]
    euclidean, bregman = scenarios["euclidean"]["controller"], scenarios["bregman"]["controller"]
    shared = all(
        np.array_equal(nominal[k], controller[k])
        for controller in (euclidean, bregman)
        for k in ("KR", "Kxi", "Lambda", "Lambda_s", "kd", "ks", "alpha")
    )
    return [{
        "Tracking gains common across payload-release comparison?": "yes" if shared else "no",
        "Lambda": np.asarray(euclidean["Lambda"]).tolist(),
        "Lambda_s": np.asarray(euclidean["Lambda_s"]).tolist(),
        "K_R": np.asarray(euclidean["KR"]).tolist(),
        "K_xi": np.asarray(euclidean["Kxi"]).tolist(),
        "k_d": float(euclidean["kd"]), "k_s": float(euclidean["ks"]), "alpha": float(euclidean["alpha"]),
        "gamma": np.asarray(euclidean["gammaE"]).tolist(), "gamma_B": float(bregman["gammaB"]),
        "Estimator-gain tuning provenance": "loaded from saved configuration; not tuned by run_paper_sim",
    }]


def gain_report():
    adaptive_scenario = paper_scenario("bregman")
    cfg = adaptive_scenario["controller"]
    return {
        "status": "saved gains reported; run_paper_sim performs no optimization",
        "adaptive_comparison_tracking_gain_source": "adaptive_base entry in optimized_gains.py; shared by known-inertia, Euclidean, and Natural/Bregman payload-release runs",
        "adaptive_comparison_common_tracking_gains": {k: cfg[k] for k in COMMON_KEYS},
        "nominal_study_tracking_gain_source": "separate payload-disabled nominal_scenario setup in publication_runner.py",
        "Lambda_s": cfg["Lambda_s"],
        "gamma": optimized_gains("euclidean", "lc")["gammaE"], "gamma_B": cfg["gammaB"],
        "optimization_executed": False,
        "tuning_protocol": {
            "paper_simulation": "loads configured gains; does not run PSO",
            "tracking_gains": "adaptive_base gains are common within the payload-release comparison; nominal validation is separate",
            "adaptation_rates": "loaded from saved configuration; this report does not establish estimator-gain tuning fairness",
        },
    }


def reaching_summary(run, scenario, epsilon=1e-8, final_time=30., time_offset=10.):
    cfg = scenario["controller"]
    inertia = inertia_from_pi(scenario["plantPi"])
    metric = np.asarray(cfg.get("Lambda_s", np.linalg.inv(cfg["Lambda"])))
    s0 = run["s"][0]
    v0 = float(.5 * s0 @ inertia @ s0)
    lo, hi = float(np.linalg.eigvalsh(metric).min()), float(np.linalg.eigvalsh(inertia).max())
    q = (1 + cfg["alpha"]) / 2
    c = 2 * lo / hi
    bound = compute_nominal_reaching_bound(run, inertia, metric, cfg["kd"], cfg["ks"], cfg["alpha"])
    observed = compute_persistent_reaching_time(run, epsilon, metric, final_time=final_time, time_offset=time_offset)
    r = np.sqrt(np.einsum("ni,ij,nj->n", run["s"], metric, run["s"]))
    loss = cfg["kd"]*r**2 + cfg["ks"]*r**(1+cfg["alpha"])
    integral = np.r_[0., np.cumsum(.5*(loss[:-1]+loss[1:])*np.diff(run["t"]))]
    true_energy = .5*np.einsum("ni,ij,nj->n", run["s"], inertia, run["s"])
    energy_residual = true_energy-v0+integral
    return {"V_s(0)": v0, "lambda_min(Lambda_s)": lo, "lambda_max(I)": hi,
            "k_d": cfg["kd"], "k_s": cfg["ks"], "alpha": cfg["alpha"], "q": q, "c_Lambda": c,
            "a": cfg["kd"]*c, "b": cfg["ks"]*c**q,
            "T_bound": bound, "T_obs": observed,
            "T_obs_source_time_s": None if observed is None else observed+time_offset,
            "epsilon_s": epsilon, "final_source_time_s": final_time,
            "max_integrated_energy_residual_J": float(np.abs(energy_residual).max()),
            "energy_residual_relative_to_initial": float(np.abs(energy_residual).max()/v0) if v0 else None,
            "passed": bool(v0 > 0 and observed is not None and observed <= bound),
            "inertia": inertia, "Lambda_s": metric, "controller": cfg}
