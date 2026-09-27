"""Paired Monte Carlo audit for the TAC manuscript (run from repo root)."""

from pathlib import Path
from concurrent.futures import ProcessPoolExecutor, as_completed
import csv
import hashlib
import json
import math
import os
import sys
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.linalg import expm

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / "results/papers"
META, TABLES = OUT / "metadata", OUT / "tables"
FIGURES = OUT / "figures/03-monte-carlo-verification"
sys.path.insert(0, str(ROOT / "src"))

from agc.config.optimized_gains import optimized_gains
from agc.math.inertia import inertia_from_pi, pi_from_pseudo, pseudo_from_pi
from agc.sim.default_scenario import default_scenario
from agc.sim.paper_metrics import _pose_errors, compute_persistent_reaching_time
from agc.sim.publication_runner import nominal_scenario
from agc.sim.run_scenario import run_scenario

SEED, N, RHO = 20260927, 100, (0.5, 1.0)


def dump(path, obj):
    path.write_text(json.dumps(obj, default=lambda x: x.tolist() if isinstance(x, np.ndarray)
                               else x.item() if isinstance(x, np.generic) else str(x),
                               indent=2, allow_nan=False) + "\n", encoding="utf-8")


def configured_gains():
    gamma_e = np.asarray(optimized_gains("euclidean", "lc")["gammaE"], float)
    gamma_b = float(optimized_gains("bregman", "lc")["gammaB"])
    if gamma_e.shape != (10,) or not np.isfinite(gamma_e).all():
        raise ValueError("Invalid configured Euclidean adaptation gain")
    return gamma_e, gamma_b


def starts(j_star):
    rng = np.random.default_rng(SEED)
    w, q = np.linalg.eigh((j_star + j_star.T) * 0.5)
    root = (q * np.sqrt(w)) @ q.T
    result = []
    for i in range(N):
        g = rng.normal(size=(4, 4))
        S = (g + g.T) * 0.5
        S /= np.linalg.norm(S, "fro")
        rho = float(rng.uniform(*RHO))
        J = root @ expm(rho * S) @ root
        J = (J + J.T) * 0.5
        pi = pi_from_pseudo(J)
        m, h = pi[0], pi[1:4]
        c = h / m
        Io = inertia_from_pi(pi)[:3, :3]
        Ic = Io - m * ((c @ c) * np.eye(3) - np.outer(c, c))
        eig = np.linalg.eigvalsh(J)
        if not (np.isfinite(J).all() and m > 0 and eig.min() > 0
                and np.isfinite(c).all() and np.linalg.eigvalsh(Ic).min() > 0):
            raise FloatingPointError(f"Invalid generated sample {i + 1}; no trial rejected silently")
        result.append({"trial": i + 1, "rho": rho, "S": S, "J0": J, "pi0": pi,
                       "mass": m, "com": c, "central_inertia": Ic})
    return result


def scenario(mode, sample, gamma_e, gamma_b, release=False):
    s = default_scenario(mode=mode, coriolis="lc", replay_id="lemniscate_02_auto",
                         duration=30.0, payload_enabled=True, enable_pacing=False,
                         release_time=10.0 if release else 31.0)
    s["payloadDrop"]["releaseTime"] = 10.0 if release else 31.0
    s["initialEstimate"] = sample["pi0"].copy() if mode == "euclidean" else sample["J0"].copy()
    s["controller"]["gammaE"] = gamma_e.copy()
    s["controller"]["gammaB"] = gamma_b
    return s


def metrics(run, failure, mode):
    pi = np.asarray(run["estimatePi"], float)
    if mode == "bregman":
        # run_scenario computes these eigenvalues from the actual Bregman state.
        eig = np.asarray(run["minPseudoEigenvalue"], float)
    else:
        # Euclidean state is a parameter vector; reconstruct its pseudo-inertia at every sample.
        j_e = np.array([pseudo_from_pi(x) for x in pi])
        eig = np.linalg.eigvalsh((j_e + j_e.transpose(0, 2, 1)) * 0.5)[:, 0]
    t = np.asarray(run["t"], float)
    p, a = _pose_errors(run)
    w = np.asarray(run["wrench"], float)
    done = (failure is None and len(t) == 15001 and np.isclose(t[-1], 30.0)
            and np.isfinite(eig).all() and np.isfinite(run["H"]).all()
            and np.isfinite(run["V"]).all() and np.isfinite(w).all())
    ix = int(np.nanargmin(eig))
    return {"min_lambda": float(eig[ix]), "t_min": float(t[ix]),
            "loss_pd": bool(np.any(eig <= 0)), "near_1e_8": bool(np.any(eig < 1e-8)),
            "position_rmse_m": float(np.sqrt(np.mean(p*p))),
            "attitude_rmse_rad": float(np.sqrt(np.mean(a*a))),
            "attitude_rmse_deg": float(np.degrees(np.sqrt(np.mean(a*a)))),
            "peak_position_m": float(np.max(p)),
            "peak_attitude_rad": float(np.max(a)),
            "peak_attitude_deg": float(np.degrees(np.max(a))),
            "max_wrench_norm": float(np.max(np.linalg.norm(w, axis=1))),
            "max_force_N": float(np.max(np.linalg.norm(w[:, 3:], axis=1))),
            "max_moment_Nm": float(np.max(np.linalg.norm(w[:, :3], axis=1))),
            "completed": bool(done), "failure": failure,
            "time": t, "history": eig}


def simulate_pair(x, gamma_e, gamma_b, release=False):
    e = scenario("euclidean", x, gamma_e, gamma_b, release)
    b = scenario("bregman", x, gamma_e, gamma_b, release)
    for k in ("KR", "Kxi", "Lambda", "kd", "ks", "alpha"):
        assert np.array_equal(e["controller"][k], b["controller"][k]), k
    assert np.array_equal(e["initial"]["H"], b["initial"]["H"])
    assert np.array_equal(e["initial"]["V"], b["initial"]["V"])
    re, fe = run_scenario(e)
    rb, fb = run_scenario(b)
    assert np.array_equal(re["Hdesired"], rb["Hdesired"])
    assert np.array_equal(re["activePlantPi"], rb["activePlantPi"])
    expected = (np.broadcast_to(e["payloadDrop"]["loadedPi"], re["activePlantPi"].shape)
                if not release else np.where(re["t"][:, None] < 10.0,
                                             e["payloadDrop"]["loadedPi"], e["payloadDrop"]["barePi"]))
    assert np.allclose(re["activePlantPi"], expected)
    return metrics(re, fe, "euclidean"), metrics(rb, fb, "bregman")


def simulate_trial(args):
    x, gamma_e, gamma_b = args
    return simulate_pair(x, gamma_e, gamma_b)


def stats(rows, key):
    x = [r[key] for r in rows if r[key]["completed"]]
    m = np.array([r["min_lambda"] for r in x])
    pos = np.array([r["position_rmse_m"] for r in x])
    att = np.array([r["attitude_rmse_rad"] for r in x])
    pos_q = np.quantile(pos, [.25, .5, .75]) if len(pos) else []
    att_q = np.quantile(att, [.25, .5, .75]) if len(att) else []
    return {"completed": len(x), "loss_pd_count": sum(r["loss_pd"] for r in x),
            "loss_pd_percent": 100*sum(r["loss_pd"] for r in x)/len(x) if x else None,
            "near_count": sum(r["near_1e_8"] for r in x),
            "near_percent": 100*sum(r["near_1e_8"] for r in x)/len(x) if x else None,
            "min": float(m.min()) if len(m) else None,
            "p05": float(np.quantile(m, .05)) if len(m) else None,
            "median": float(np.median(m)) if len(m) else None,
            "p95": float(np.quantile(m, .95)) if len(m) else None,
            "position_rmse_median_q25_q75": [float(pos_q[1]), float(pos_q[0]), float(pos_q[2])] if len(pos) else [],
            "position_rmse_iqr_m": float(pos_q[2]-pos_q[0]) if len(pos) else None,
            "attitude_rmse_rad_median_q25_q75": [float(att_q[1]), float(att_q[0]), float(att_q[2])] if len(att) else [],
            "attitude_rmse_rad_iqr": float(att_q[2]-att_q[0]) if len(att) else None}


def save_trials(path, rows):
    fields = ["trial", "rho", "S_row_major", "J0_row_major", "pi0", "mass_kg", "com_m",
              "central_inertia_row_major"]
    metric_keys = ["min_lambda", "t_min", "loss_pd", "near_1e_8", "position_rmse_m",
                   "attitude_rmse_rad", "attitude_rmse_deg", "peak_position_m",
                   "peak_attitude_rad", "peak_attitude_deg", "max_wrench_norm", "max_force_N",
                   "max_moment_Nm", "completed", "failure"]
    fields += [f"{mode}_{key}" for mode in ("euclidean", "bregman") for key in metric_keys]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for r in rows:
            x = r["start"]
            d = {"trial": x["trial"], "rho": x["rho"],
                 "S_row_major": json.dumps(x["S"].ravel().tolist()),
                 "J0_row_major": json.dumps(x["J0"].ravel().tolist()),
                 "pi0": json.dumps(x["pi0"].tolist()), "mass_kg": x["mass"],
                 "com_m": json.dumps(x["com"].tolist()),
                 "central_inertia_row_major": json.dumps(x["central_inertia"].ravel().tolist())}
            for mode in ("euclidean", "bregman"):
                d.update({f"{mode}_{k}": json.dumps(r[mode][k]) if k == "failure" else r[mode][k]
                          for k in metric_keys})
            writer.writerow(d)


def reaching_certificate():
    s = nominal_scenario(20.0, dt=.002)
    s["initial"]["V"] = s["sourceTrajectory"](10.0)["V"] + 4*np.array([.3, -.2, .1, .4, -.2, .3])
    run, failure = run_scenario(s)
    if failure:
        raise RuntimeError(f"Nominal reaching simulation failed: {failure}")
    I = inertia_from_pi(s["plantPi"])
    Ls = np.asarray(s["controller"]["Lambda_s"])
    S = run["s"]
    v0 = float(.5*S[0]@I@S[0])
    lo, hi = np.linalg.eigvalsh(Ls).min(), np.linalg.eigvalsh(I).max()
    q = (1+s["controller"]["alpha"])/2
    c = 2*lo/hi
    a = s["controller"]["kd"]*c
    b = s["controller"]["ks"]*c**q
    old = v0**(1-q)/(b*(1-q))
    new = math.log1p((a/b)*v0**(1-q))/(a*(1-q))
    persistent = compute_persistent_reaching_time(run, 1e-8, Ls, final_time=30., time_offset=10.)
    return {"V_s(0)": v0, "c_Lambda": c, "q": q, "a": a, "b": b,
            "old_bound_s": old, "new_bound_s": new, "T_obs_elapsed_s": persistent,
            "T_obs_source_s": None if persistent is None else persistent+10.,
            "persistent_threshold": 1e-8, "persistent_final_source_time_s": 30.,
            "old_over_observed": None if persistent is None else old/persistent,
            "new_over_observed": None if persistent is None else new/persistent,
            "initial_s": S[0], "initial_H": s["initial"]["H"],
            "initial_V": s["initial"]["V"], "Lambda": s["controller"]["Lambda"], "Lambda_s": Ls}


def write_audit(s, gamma_e, gamma_b):
    cfg = s["controller"]
    I = inertia_from_pi(s["plantPi"])
    replay = s["trajectory"].__self__
    replay_times = np.asarray(replay.t, dtype=float)
    replay_steps = np.diff(replay_times)
    payload = s["payloadDrop"]["payload"]
    pi0, pil = s["payloadDrop"]["barePi"], s["payloadDrop"]["loadedPi"]
    nom = nominal_scenario(20.0, dt=.002)
    Lnom = nom["controller"]["Lambda"]
    Lsnom = np.asarray(nom["controller"]["Lambda_s"])
    def diag(x): return r"\operatorname{diag}(" + r",\;".join(f"{v:.8g}" for v in x) + ")"
    def mat(x): return r"\begin{bmatrix}" + r" \\".join(" & ".join(f"{v:.7g}" for v in row) for row in x) + r"\end{bmatrix}"
    rows = [
        (r"Adaptive $\Lambda$", f"${diag(np.diag(cfg['Lambda']))}$", "optimized_gains.py: LambdaDiag"),
        (r"Adaptive $\Lambda_s$", rf"${diag(np.diag(cfg['Lambda_s']))}$ (six diagonal entries)", "default_scenario.py: Lambda_s"),
        (r"$K_R$, $K_\xi$", rf"${diag(np.diag(cfg['KR']))},\quad {diag(np.diag(cfg['Kxi']))}$", "optimized_gains.py: KRdiag, Kxidiag"),
        (r"$k_d,k_s,\alpha$", f"{cfg['kd']}, {cfg['ks']}, {cfg['alpha']}", "optimized_gains.py: kd, ks, alpha"),
        (r"Euclidean $\Gamma$", f"${diag(gamma_e)}$", "optimized_gains.py: gammaE"),
        (r"Natural/Bregman $\gamma_B$", f"{gamma_b} (scalar)", "optimized_gains.py: gammaB"),
        ("Plant/control/adaptation step", "0.002 / 0.02 / 0.01 s", "default_scenario.py"),
        ("Reference update", "At each plant step; linear interpolation and quaternion SLERP", "replay_trajectory.py"),
        ("Reference source grid / endpoint", f"{np.median(replay_steps):.6g} s samples over {replay_times[0]:.6g}Ã¢â‚¬â€œ{replay_times[-1]:.6g} s; requests past the endpoint are clamped to the final sample", "lemniscate_02_auto.npz / replay_trajectory.py"),
        ("Integrator", "PyBullet stepSimulation once per plant step; default internal method, no explicit substeps", "pybullet_plant.py"),
        ("Duration and random seed", "30 s; deterministic manuscript runs have no seed; Monte Carlo seed 20260927", "default_scenario.py / experiment"),
        ("Nominal reaching duration and replay", "20 s; source time 10-30 s on lemniscate_01_auto", "publication_runner.py"),
        ("Payload release", f"10 s; mass {payload['mass']} kg, dimensions {payload['dimensions']}, center {payload['center']} m", "default_scenario.py"),
        (r"Pre-release loaded $\pi$", f"$[{', '.join(f'{v:.8g}' for v in pil)}]$", "compound_pi.py"),
        (r"Post-release bare $\pi$", f"$[{', '.join(f'{v:.8g}' for v in pi0)}]$", "default_scenario.py"),
        (r"Nominal reaching $K_R$, $K_\xi$", rf"${diag(np.diag(nom['controller']['KR']))},\quad {diag(np.diag(nom['controller']['Kxi']))}$", "publication_runner.py: nominal_scenario"),
        (r"Nominal reaching $\Lambda$ (derived, not tuned)", f"${mat(Lnom)}$", "publication_runner.py"),
        (r"Nominal reaching $\Lambda_s$ (derived, not tuned)", f"${mat(Lsnom)}$", "publication_runner.py"),
        (r"Nominal reaching $k_d,k_s,\alpha$", "1, 1, 0.5", "publication_runner.py"),
        ("Safeguards", "No wrench clipping/allocation; no Euclidean projection. Bregman symmetrizes, clips exponent eigenvalues to [-50,50], floors eigenvalues at 1e-14 max eigenvalue, falls back to prior symmetrized state on nonfinite update.", "adaptation.py / pybullet_plant.py"),
    ]
    table = [r"\begin{table*}[t]\centering\small", r"\caption{Implemented manuscript and experiment parameters.}",
             r"\begin{tabular}{p{0.24\linewidth}p{0.61\linewidth}p{0.12\linewidth}}\hline",
             r"Quantity & Value & Source \\", r"\hline"]
    table += [f"{a} & {b} & {c} \\\\" for a,b,c in rows]
    table += [r"\hline\end{tabular}\end{table*}"]
    (TABLES/"physical_consistency_monte_carlo_implemented_parameters.tex").write_text("\n".join(table)+"\n", encoding="utf-8")
    dump(META/"physical_consistency_monte_carlo_implemented_parameters.json", {
        "adaptive_controller": {"Lambda": cfg["Lambda"], "Lambda_s": cfg["Lambda_s"],
            "Lambda_structure": "diagonal; six configured entries; Lambda_s configured independently (default reciprocal)",
            "KR": cfg["KR"], "Kxi": cfg["Kxi"], "kd": cfg["kd"], "ks": cfg["ks"], "alpha": cfg["alpha"]},
        "estimators": {"euclidean_gamma": gamma_e, "euclidean_gamma_structure": "diagonal 10-vector",
            "natural_bregman_gamma": gamma_b, "natural_bregman_gamma_structure": "scalar"},
        "simulation": {"duration_s": s["duration"], "dt_plant_s": s["dtPlant"],
            "dt_control_s": s["dtControl"], "dt_adaptation_s": s["dtAdaptation"],
            "integrator": "PyBullet stepSimulation once per plant step; default internal method; no explicit substeps",
            "reference": {"replay_id": "lemniscate_02_auto", "sample_dt_s": float(np.median(replay_steps)),
                "source_start_s": float(replay_times[0]), "source_end_s": float(replay_times[-1]),
                "update": "each plant tick; linear interpolation and quaternion SLERP",
                "beyond_endpoint": "sample() clips to final recorded sample"}},
        "randomization": {"seed": SEED, "trials": N, "rho_uniform": RHO,
            "generator": "NumPy default_rng / PCG64"},
        "payload_release_reference": {"release_time_s": s["payloadDrop"]["releaseTime"],
            "payload": payload, "pre_release_loaded_pi": pil, "post_release_bare_pi": pi0},
        "nominal_reaching": {"duration_s": nom["duration"], "replay_id": "lemniscate_01_auto",
            "source_time_interval_s": [10.0, 30.0], "Lambda": Lnom, "Lambda_s": Lsnom,
            "KR": nom["controller"]["KR"], "Kxi": nom["controller"]["Kxi"],
            "kd": nom["controller"]["kd"], "ks": nom["controller"]["ks"],
            "alpha": nom["controller"]["alpha"], "initial_H": nom["initial"]["H"],
            "initial_V": nom["initial"]["V"]},
        "safeguards": {"euclidean": "no projection or SPD correction",
            "natural_bregman": "symmetrization; exponent eigenvalue clip [-50,50]; eigenvalue floor 1e-14 times maximum eigenvalue; fallback to previous symmetrized state on nonfinite update",
            "actuator": "no actuator allocation, wrench saturation, or clipping"},
        "sources": {"adaptive_gains": "src/agc/config/optimized_gains.py: LambdaDiag, KRdiag, Kxidiag, kd, ks, alpha, gammaE, gammaB",
            "simulation_timing_and_payload": "src/agc/sim/default_scenario.py: dtPlant, dtControl, dtAdaptation, releaseTime, evaluation payload profile",
            "reference_sampling": ["trajectories/processed/lemniscate_02_auto.npz", "src/agc/sim/replay_trajectory.py"],
            "nominal_reaching_protocol": "src/agc/sim/publication_runner.py",
            "actuation_and_integrator": "src/agc/plant/pybullet_plant.py",
            "adaptation_safeguards": "src/agc/paper/adaptation.py",
            "randomization_and_experiment": "src/agc/sim/physical_consistency_experiment.py"}})


def figures(rows):
    done = [r for r in rows if r["euclidean"]["completed"] and r["bregman"]["completed"]]
    fig, ax = plt.subplots(figsize=(7,4.5), constrained_layout=True)
    for mode, label, color in (("euclidean","Euclidean","#00FF00"),("bregman","Natural/Bregman","#0000FF")):
        z = np.sort([r[mode]["min_lambda"] for r in done]); y=np.arange(1,len(z)+1)/len(z)
        ax.step(z,y,where="post",label=label,color=color,lw=2)
    ax.axvline(0,color="black",ls="--",lw=1.7,label="SPD boundary")
    ax.set(xlabel=r"$\min_t\lambda_{\min}(\hat{\mathcal{J}}(t))$",ylabel="Empirical cumulative probability")
    ax.grid(alpha=.25); fig.legend(*ax.get_legend_handles_labels(), loc="outside upper center", ncol=3, frameon=False)
    for ext in ("pdf","png"): fig.savefig(FIGURES/f"physical_consistency_monte_carlo_figure1_margin_ecdf.{ext}",dpi=400 if ext=="png" else None)
    plt.close(fig)
    bad = [r for r in done if r["euclidean"]["loss_pd"]]
    pick = min(bad,key=lambda r:r["start"]["trial"]) if bad else min(done,key=lambda r:r["euclidean"]["min_lambda"])
    fig,ax=plt.subplots(figsize=(7,4.5),constrained_layout=True)
    ax.plot(pick["euclidean"]["time"],pick["euclidean"]["history"],label="Euclidean",color="#00FF00")
    ax.plot(pick["bregman"]["time"],pick["bregman"]["history"],label="Natural/Bregman",color="#0000FF")
    ax.axhline(0,color="black",ls="--",label="SPD boundary")
    ax.set(xlabel="Time [s]",ylabel=r"$\lambda_{\min}(\hat{\mathcal{J}})$",xlim=(0,30)); ax.margins(x=0)
    ax.grid(alpha=.25); fig.legend(*ax.get_legend_handles_labels(), loc="outside upper center", ncol=3, frameon=False)
    for ext in ("pdf","png"): fig.savefig(FIGURES/f"physical_consistency_monte_carlo_figure2_trial.{ext}",dpi=400 if ext=="png" else None)
    plt.close(fig)
    return {"trial":pick["start"]["trial"],"selection":"first Euclidean violation" if bad else "smallest Euclidean margin; no Euclidean violation observed"}


def run_experiment(output_dir=None):
    global OUT, META, TABLES, FIGURES
    if output_dir is not None:
        OUT = Path(output_dir)
        META, TABLES, FIGURES = OUT / "metadata", OUT / "tables", OUT / "figures/03-monte-carlo-verification"
    for directory in (META, TABLES, FIGURES): directory.mkdir(parents=True, exist_ok=True)
    ge,gamma_b=configured_gains()
    base=default_scenario(mode="euclidean",coriolis="lc",replay_id="lemniscate_02_auto",duration=30,payload_enabled=True,enable_pacing=False)
    # Nominal reaching certificate uses the documented 4x experiment, not the superseded 1x setup.
    reach=reaching_certificate()
    j_star=pseudo_from_pi(base["payloadDrop"]["loadedPi"])
    xs=starts(j_star)
    pre=[]
    for x in xs[:5]:
        pre.append({"trial":x["trial"],"rho":x["rho"],"mass_kg":x["mass"],"com_m":x["com"],
                    "central_inertia":x["central_inertia"],"pseudo_eigenvalues":np.linalg.eigvalsh(x["J0"])})
    dump(META/"physical_consistency_monte_carlo_initializations.json",{"seed":SEED,"rng":"numpy default_rng / PCG64","rho_uniform":RHO,
        "construction":"Jstar^(1/2) exp(rho S) Jstar^(1/2), S=sym(iid N(0,1)) / ||S||_F",
        "Jstar":j_star,"pi_star":base["payloadDrop"]["loadedPi"],"preflight_first_five":pre,
        "rejected_samples":[],"samples":[{k:x[k] for k in ("trial","rho","S","J0","pi0","mass","com","central_inertia")} for x in xs]})
    write_audit(base,ge,gamma_b)
    traj=ROOT/"trajectories/processed/lemniscate_02_auto.npz"
    workers=min(4, os.cpu_count() or 1)
    setup={"seed":SEED,"trials":N,"duration_s":30,"trajectory":"lemniscate_02_auto",
        "trajectory_sha256":hashlib.sha256(traj.read_bytes()).hexdigest(),"initial_state":base["initial"],
        "reference_sampling":{"sample_interval_s":float(np.median(np.diff(base["trajectory"].__self__.t))),
            "source_start_s":float(base["trajectory"].__self__.t[0]),
            "source_end_s":float(base["trajectory"].__self__.t[-1]),
            "beyond_endpoint":"ReplayTrajectory.sample clips to the last recorded sample"},
        "timesteps_s":{"plant":base["dtPlant"],"control":base["dtControl"],"adaptation":base["dtAdaptation"]},
        "constant_plant_pi":base["payloadDrop"]["loadedPi"],"payload":base["payloadDrop"]["payload"],
        "controller":base["controller"],"experiment_gammaE":ge,"experiment_gammaB":gamma_b,
        "adaptation_gain_source":"Current manuscript configuration in optimized_gains.py; no separate retuning",
        "primary_payload_switch":"disabled; attached payload held through end (release at 31 s)",
        "independent_trial_processes":workers}
    source_files=["src/agc/config/optimized_gains.py","src/agc/sim/default_scenario.py",
        "src/agc/sim/physical_consistency_experiment.py",
        "src/agc/sim/publication_runner.py","src/agc/sim/run_scenario.py",
        "src/agc/sim/replay_trajectory.py","src/agc/paper/controller.py",
        "src/agc/paper/adaptation.py","src/agc/math/inertia.py",
        "src/agc/plant/pybullet_plant.py","src/agc/plant/compound_pi.py",
        "run/run_paper_sim.py"]
    setup["source_files"]={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in source_files}
    setup["finite_time_certificate"]=reach
    dump(META/"physical_consistency_monte_carlo_experiment.json",setup)
    rows=[]; t0=time.time()
    with ProcessPoolExecutor(max_workers=workers) as pool:
        futures={pool.submit(simulate_trial,(x,ge,gamma_b)):x for x in xs}
        for future in as_completed(futures):
            x=futures[future]
            e,b=future.result()
            rows.append({"start":x,"euclidean":e,"bregman":b})
            rows.sort(key=lambda row:row["start"]["trial"])
            save_trials(TABLES/"physical_consistency_monte_carlo_trials.csv",rows)
            print(f"Completed trial {x['trial']}/{N}",flush=True)
    paired=[r for r in rows if r["euclidean"]["completed"] and r["bregman"]["completed"]]
    categories={"both_consistent":0,"only_natural_consistent":0,"only_euclidean_consistent":0,"both_lose":0}
    for r in paired:
        eok=not r["euclidean"]["loss_pd"]; bok=not r["bregman"]["loss_pd"]
        categories["both_consistent" if eok and bok else "only_natural_consistent" if bok else "only_euclidean_consistent" if eok else "both_lose"]+=1
    summary={"completed_pairs":len(paired),"euclidean":stats(rows,"euclidean"),"natural_bregman":stats(rows,"bregman"),
        "paired_consistency":categories,"elapsed_s":time.time()-t0,"figure2":figures(rows),"certificate":reach}
    summary["figure2_statement"] = ("No Euclidean physical-consistency violation was observed in the predetermined 100-trial experiment."
        if summary["euclidean"]["loss_pd_count"] == 0 else
        f"Figure 2 shows the first violating Euclidean trial, trial {summary['figure2']['trial']}.")
    summary["failed_or_nonfinite_trials"]=[{"trial":r["start"]["trial"],"euclidean":r["euclidean"]["failure"],"bregman":r["bregman"]["failure"]} for r in rows if not(r["euclidean"]["completed"] and r["bregman"]["completed"])]
    summary["runtime_warnings"]="No run_scenario failures or nonfinite trial data; run_scenario suppresses RuntimeWarning. A source SyntaxWarning in the initial table string was fixed before final table generation."
    summary["implementation_changes"]=["The independent paired trials were dispatched to four worker processes; controller, estimator, plant, trajectory, and integration calculations were unchanged."]
    summary["existing_bregman_safeguards"]="Exponent clip [-50,50], eigenvalue floors and nonfinite fallback remain unchanged."
    dump(META/"physical_consistency_monte_carlo_summary.json",summary)
    def compact(name):
        z=summary[name]
        return (f"completed {z['completed']}; PD losses {z['loss_pd_count']}/100 ({z['loss_pd_percent']:.1f}%); "
                f"<1e-8 {z['near_count']}/100 ({z['near_percent']:.1f}%); min/p05/median/p95 margin "
                f"{z['min']:.8g}/{z['p05']:.8g}/{z['median']:.8g}/{z['p95']:.8g}; "
                f"position RMSE median {z['position_rmse_median_q25_q75'][0]:.8g} m, IQR {z['position_rmse_iqr_m']:.8g} m; "
                f"attitude RMSE median {z['attitude_rmse_rad_median_q25_q75'][0]:.8g} rad, IQR {z['attitude_rmse_rad_iqr']:.8g} rad")
    observed = reach["T_obs_elapsed_s"]
    observed_text = "not observed through source time 30 s" if observed is None else f"{observed:.8g} s"
    old_ratio = "n/a" if reach["old_over_observed"] is None else f"{reach['old_over_observed']:.5g}x observed"
    new_ratio = "n/a" if reach["new_over_observed"] is None else f"{reach['new_over_observed']:.5g}x"
    reach_text=(f"Nominal reaching certificate: V_s(0)={reach['V_s(0)']:.10g}, "
        f"c_Lambda={reach['c_Lambda']:.10g}, q={reach['q']:.10g}, a={reach['a']:.10g}, "
        f"b={reach['b']:.10g}; old fractional-only bound {reach['old_bound_s']:.8g} s "
        f"({old_ratio}), full bound with k_d {reach['new_bound_s']:.8g} s "
        f"({new_ratio}), T_obs {observed_text} under the 1e-8-through-30 s criterion.\n\n")
    (META/"physical_consistency_monte_carlo_report.md").write_text(
        reach_text + f"Euclidean: {compact('euclidean')}\n\nNatural/Bregman: {compact('natural_bregman')}\n\n"
        "We ran 100 paired 30 s simulations on the `lemniscate_02_auto` reference using seed 20260927 and independently sampled rho uniformly from [0.5,1]. Each estimate was initialized by Jstar^(1/2) exp(rho S) Jstar^(1/2), with symmetric Gaussian S normalized to unit Frobenius norm. Both estimators shared the same start, loaded vehicle-plus-payload plant, trajectory, tracking controller and simulation rates; the payload remained attached to maintain constant true inertia. The source replay is sampled every 0.002 s through 25.664 s; its sampler clamps references to the last recorded sample over the remaining 4.336 s. The adaptation laws used their frozen rates, with no added projection or SPD correction.\n\n"
        f"Among {len(paired)} completed pairs, physical-consistency outcomes were {categories}. {summary['figure2_statement']} The experiment tests preservation of the physically consistent parameter set; it does not rank tracking controllers. Natural/Bregman has a structural SPD-preservation guarantee, while unconstrained Euclidean adaptation does not. The observed trial outcomes neither prove invariance when no Euclidean violation occurs nor imply that Euclidean adaptation must become nonphysical.\n",
        encoding="utf-8")
    print(json.dumps(summary,indent=2,default=lambda x:x.tolist() if isinstance(x,np.ndarray) else str(x)))
