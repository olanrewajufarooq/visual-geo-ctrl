"""Publication protocol: mode-specific gains, explicit units, and auditable validation.

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

NAMES = {"nominal": "Known-inertia controller", "euclidean": "Euclidean adaptive controller",
         "bregman": "Natural/Bregman adaptive controller"}
ADAPTIVE_MODES = ("euclidean", "bregman")
PRIMARY_MODES = ("nominal",) + ADAPTIVE_MODES
CONNECTIONS = ("lc", "rb")
CONNECTION_PERSISTENCE_THRESHOLD = 1e-4
# Sections 01--03 are single-realization studies.  The explicit LC/RB
# comparison is reserved for the nominal-validation protocol in section 04.
# Keeping this registry LC-only prevents the adaptive and physical-consistency
# figures from being interpreted as connection comparisons.
PAPER_RUNS = tuple(f"{mode}_lc" for mode in PRIMARY_MODES)


def paper_scenario(mode, duration=30., coriolis="lc"):
    scenario = default_scenario(mode=mode, duration=duration, coriolis=coriolis, enable_pacing=False)
    if mode.lower() == "nominal":
        # This is a mode-specific nominal gain set. The separate nominal
        # connection/reaching validation uses its own no-payload scenario.
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


def baseline_metrics(run, mode, failure=None, coriolis="lc"):
    t = run["t"]
    position, attitude = _pose_errors(run)
    attitude = np.degrees(attitude)
    row = {"Controller": NAMES[mode], "Coriolis": coriolis.upper(),
           "Status": "failed / partial data" if failure else "completed"}
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


def adaptive_performance_row(run, mode, failure=None, coriolis="lc"):
    """Paper table row with controller-facing names and separate effort units."""
    row = baseline_metrics(run, mode, failure, coriolis)
    return {
        "Controller": row["Controller"],
        "Coriolis": row["Coriolis"],
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


def physical_consistency_row(run, mode, coriolis="lc"):
    if mode not in ADAPTIVE_MODES:
        raise ValueError("Physical-consistency table is defined for adaptive estimators only")
    margin = np.asarray(run["minPseudoEigenvalue"], dtype=float)
    t = np.asarray(run["t"], dtype=float)
    post_release = margin[t >= 10.]
    return {
        "Controller": NAMES[mode],
        "Coriolis": coriolis.upper(),
        "Initial estimated mass [kg]": float(run["estimatePi"][0, 0]),
        "Final estimated mass [kg]": float(run["estimatePi"][-1, 0]),
        "Minimum lambda_min(Jhat)": float(np.min(margin)),
        "Minimum post-release lambda_min(Jhat)": float(np.min(post_release)) if len(post_release) else None,
        "Physical-consistency violation?": "yes" if bool(np.any(margin <= 0.)) else "no",
    }


def connection_realization_row(run, connection, scenario, epsilon=CONNECTION_PERSISTENCE_THRESHOLD, final_time=30., time_offset=10.):
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
    """Machine-readable per-controller gains actually used in the saved runs."""
    rows = []
    for run_key in PAPER_RUNS:
        if run_key not in scenarios:
            continue
        mode, coriolis = run_key.rsplit("_", 1)
        cfg = scenarios[run_key]["controller"]
        rows.append({
            "Mode": mode,
            "Controller": NAMES[mode],
            "Gain set": f"{mode}_{coriolis}",
            "Coriolis": cfg["coriolis"].upper(),
            "K_R": np.asarray(cfg["KR"]).tolist(),
            "K_xi": np.asarray(cfg["Kxi"]).tolist(),
            "Lambda": np.asarray(cfg["Lambda"]).tolist(),
            "Lambda_s": np.asarray(cfg["Lambda_s"]).tolist(),
            "k_d": float(cfg["kd"]),
            "k_s": float(cfg["ks"]),
            "alpha": float(cfg["alpha"]),
            "gamma_E": np.asarray(cfg["gammaE"]).tolist(),
            "gamma_B": float(cfg["gammaB"]),
            "Gain provenance": f"optimized_gains.py:{mode}_{cfg['coriolis']}; run_paper_sim does not optimize",
        })
    return rows


def gain_report():
    scenarios = {}
    for key in PAPER_RUNS:
        mode, coriolis = key.rsplit("_", 1)
        scenarios[key] = paper_scenario(mode, coriolis=coriolis)
    gain_sources = {key: f"{key} optimized gain entry" for key in PAPER_RUNS}
    gain_sets = {
        key: {
            "K_R": scenario["controller"]["KR"],
            "K_xi": scenario["controller"]["Kxi"],
            "Lambda": scenario["controller"]["Lambda"],
            "Lambda_s": scenario["controller"]["Lambda_s"],
            "k_d": scenario["controller"]["kd"],
            "k_s": scenario["controller"]["ks"],
            "alpha": scenario["controller"]["alpha"],
            "gamma_E": scenario["controller"]["gammaE"],
            "gamma_B": scenario["controller"]["gammaB"],
        }
        for key, scenario in scenarios.items()
    }
    return {
        "status": "saved gains reported; run_paper_sim performs no optimization",
        "adaptive_comparison_gain_sources": gain_sources,
        "adaptive_comparison_gain_sets": gain_sets,
        "nominal_study_tracking_gain_source": "separate payload-disabled nominal_scenario setup in publication_runner.py",
        "optimization_executed": False,
        "tuning_protocol": {
            "paper_simulation": "loads configured gains; does not run PSO",
            "tracking_gains": "nominal, Euclidean, and Bregman payload-release controllers use their respective LC gain entries; Coriolis comparison is reserved for nominal validation",
            "adaptation_rates": "loaded from each mode's saved configuration; this report does not establish estimator-gain tuning fairness",
        },
    }


def reaching_summary(run, scenario, epsilon=CONNECTION_PERSISTENCE_THRESHOLD, final_time=30., time_offset=10.):
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
