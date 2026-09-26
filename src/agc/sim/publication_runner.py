"""Run the publication protocol without optimization or deterministic repeats."""
from pathlib import Path
import hashlib
import json
import subprocess
import numpy as np
from .publication import (paper_scenario, gain_report, baseline_metrics, adaptive_performance_row,
                           physical_consistency_row, controller_gain_rows, connection_realization_row,
                           write_json, write_csv, reaching_summary, NAMES, PRIMARY_MODES, ADAPTIVE_MODES)
from .default_scenario import default_scenario
from .run_scenario import run_scenario
from ..io.persistence import save_run, load_run
from ..paper.diagnostics import connection_identity
from ..math.inertia import inertia_from_pi
from ..viz.publication_figures import adaptive_figures, theory_figures, connection_realization_figures


def nominal_scenario(duration, dt=.002):
    scenario = default_scenario(mode="nominal", payload_enabled=False, duration=duration, enable_pacing=False)
    source = scenario["trajectory"]
    phase = 10.
    available_duration = float(source.__self__.t[-1] - phase)
    scenario["timeOffset"] = phase
    scenario["displayEnd"] = 30.0
    scenario["requestedDuration"] = float(duration)
    scenario["duration"] = min(float(duration), available_duration)
    desired_at_release = source(phase)
    scenario["sourceTrajectory"] = source
    scenario["trajectory"] = lambda t: source(t + phase)
    scenario["replayId"] = f"{scenario['replayId']}@payload-release-phase-10s"
    scenario["initial"] = {
        "H": desired_at_release["H"].copy(),
        "V": desired_at_release["V"].copy() + np.array([.3, -.2, .1, .4, -.2, .3]),
    }
    inertia = inertia_from_pi(scenario["plantPi"])
    scenario["controller"].update(KR=np.diag([1., 2., 3.]), Kxi=np.eye(3),
                                   Lambda=np.linalg.inv(inertia), kd=1., ks=1., alpha=.5)
    scenario.update(dtPlant=dt, dtControl=dt, dtAdaptation=dt)
    return scenario


def connection_test(run, scenario):
    indices = np.arange(0, len(run["t"]), max(1, len(run["t"])//1000))
    norms = []; raw = []
    for i in indices:
        result = connection_identity({"H": run["H"][i], "V": run["V"][i]},
            {"H": run["Hdesired"][i], "V": run["Vdesired"][i], "Vdot": run["VdotDesired"][i]},
            scenario["controller"], scenario["plantPi"])
        # Explicit nondimensionalization: torque / 1 N m; force / 1 N.
        norms.append([result[k] for k in ("differenceNorm", "theoreticalNorm", "residualNorm")])
        raw.append(result["residual"])
    values = np.array(norms)
    scales = np.maximum(values[:, 0], values[:, 1])
    # Relative error becomes ill-conditioned when subtracting near-identical
    # gravity-compensating wrenches. Retain absolute checks at every sample.
    meaningful = scales > 1e-6
    rel = float(np.max(values[meaningful, 2]/scales[meaningful])) if meaningful.any() else None
    metric = np.linalg.inv(scenario["controller"]["Lambda"])
    r = np.sqrt(np.einsum("ni,ij,nj->n", run["s"], metric, run["s"]))
    i = int(indices[min(5, len(indices)-1)])
    on_manifold = connection_identity({"H": run["H"][i], "V": run["V"][i]-run["s"][i]},
        {"H": run["Hdesired"][i], "V": run["Vdesired"][i], "Vdot": run["VdotDesired"][i]},
        scenario["controller"], scenario["plantPi"])
    return {"t": run["t"][indices], "difference": values[:, 0], "theory": values[:, 1], "residual": values[:, 2],
            "max_residual": float(values[:, 2].max()), "rms_residual": float(np.sqrt(np.mean(values[:, 2]**2))),
            "max_relative_residual": rel, "relative_denominator_floor": 1e-6,
            "relative_excluded_samples": int((~meaningful).sum()),
            "maximum_wrench_difference": float(values[:, 0].max()),
            "max_torque_residual_Nm": float(np.linalg.norm(np.array(raw)[:, :3], axis=1).max()),
            "max_force_residual_N": float(np.linalg.norm(np.array(raw)[:, 3:], axis=1).max()),
            "initial_wrench_difference": float(values[0, 0]), "final_wrench_difference": float(values[-1, 0]),
            "initial_weighted_s_norm": float(r[0]), "final_weighted_s_norm": float(r[-1]),
            "constructed_on_manifold_wrench_difference": on_manifold["differenceNorm"],
            "norm_definition": "Euclidean norm of [torque/(1 N m), force/(1 N)]",
            "passed": bool(np.all(values[:, 2] <= 1e-12 + 1e-10*scales) and on_manifold["differenceNorm"] < 1e-10)}


def _json_ready(value):
    if isinstance(value, np.ndarray): return value.tolist()
    if isinstance(value, np.generic): return value.item()
    if isinstance(value, dict): return {str(k): _json_ready(v) for k, v in value.items() if not callable(v)}
    if isinstance(value, (list, tuple)): return [_json_ready(v) for v in value]
    return value


def source_fingerprint():
    repo = Path(__file__).resolve().parents[3]
    source_files = sorted((repo/"src").rglob("*.py")) + [repo/"run"/"run_paper_sim_figures.py"]
    digest = hashlib.sha256()
    for path in source_files:
        digest.update(str(path.relative_to(repo)).replace("\\", "/").encode())
        digest.update(path.read_bytes())
    return digest.hexdigest()


def scenario_fingerprint(scenario):
    duration = float(scenario["duration"])
    sample_times = sorted(set((0., min(10., duration), duration)))
    reference = []
    for time in sample_times:
        desired = scenario["trajectory"](time)
        reference.append({"time": time, "H": desired["H"], "V": desired["V"], "Vdot": desired.get("Vdot", np.zeros(6))})
    payload = {key: scenario.get(key) for key in ("duration", "dtPlant", "dtControl", "dtAdaptation", "plantPi", "plantGravity", "payloadDrop", "initial", "initialEstimate", "controller", "replayId", "payloadProfile", "requestedDuration")}
    payload["reference_samples"] = reference
    return hashlib.sha256(json.dumps(_json_ready(payload), sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def cached_scenario(run):
    """Recover the numerical controller configuration saved with a raw run."""
    metadata = run["metadata"]
    controller = {}
    for key, value in metadata["controller"].items():
        controller[key] = np.asarray(value, dtype=float) if isinstance(value, list) else value
    replay_id = metadata.get("replayId", "unknown")
    is_release_phase = "@payload-release-phase-10s" in replay_id
    return {"controller": controller, "plantPi": np.asarray(metadata["plantPi"], dtype=float),
            "plantGravity": np.asarray(metadata["plantGravity"], dtype=float),
            "payloadDrop": metadata.get("payloadDrop"), "duration": metadata["timing"]["duration"],
            "dtPlant": metadata["timing"]["dtPlant"], "dtControl": metadata["timing"]["dtControl"],
            "dtAdaptation": metadata["timing"]["dtAdaptation"], "replayId": replay_id,
            "timeOffset": 10.0 if is_release_phase else 0.0,
            "displayEnd": 30.0 if is_release_phase else metadata["timing"]["duration"]}


def load_or_run(path, scenario, reuse_cache=False):
    path = Path(path)
    cache = {"scenario_sha256": scenario_fingerprint(scenario), "source_sha256": source_fingerprint()}
    if reuse_cache and (path / "run.npz").is_file():
        run = load_run(str(path))
        stored = run.get("metadata", {}).get("cache")
        completed_duration = float(run["t"][-1]) if len(run.get("t", [])) else None
        if stored == cache and completed_duration is not None and np.isclose(completed_duration, float(scenario["duration"])):
            return run, run.get("failure"), True
    run, failure = run_scenario(scenario)
    save_run(str(path), run, None, scenario, failure, cache_metadata=cache)
    return load_run(str(path)), failure, False


def connection_pair_protocol(lc_run, rb_run):
    lc_meta, rb_meta = lc_run["metadata"], rb_run["metadata"]
    lc_controller, rb_controller = dict(lc_meta["controller"]), dict(rb_meta["controller"])
    lc_controller.pop("coriolis", None); rb_controller.pop("coriolis", None)
    exact_known = (lc_meta.get("mode") == "nominal" and rb_meta.get("mode") == "nominal"
                   and np.array_equal(lc_meta.get("initialEstimate"), lc_meta.get("plantPi"))
                   and np.array_equal(rb_meta.get("initialEstimate"), rb_meta.get("plantPi")))
    payload_release = bool(lc_meta.get("payloadDrop") is not None or rb_meta.get("payloadDrop") is not None)
    adaptation = bool(lc_meta.get("mode") != "nominal" or rb_meta.get("mode") != "nominal")
    same_controller = lc_controller == rb_controller
    same_initial = lc_meta.get("initial") == rb_meta.get("initial")
    same_initial_estimate = lc_meta.get("initialEstimate") == rb_meta.get("initialEstimate")
    same_plant = lc_meta.get("plantPi") == rb_meta.get("plantPi") and lc_meta.get("plantGravity") == rb_meta.get("plantGravity")
    same_timing = lc_meta.get("timing") == rb_meta.get("timing") and lc_meta.get("replayId") == rb_meta.get("replayId")
    desired_samples = bool(np.array_equal(lc_run["Hdesired"], rb_run["Hdesired"]) and np.array_equal(lc_run["Vdesired"], rb_run["Vdesired"]))
    only_intended = bool(exact_known and not payload_release and not adaptation and lc_meta.get("coriolis") == "lc" and rb_meta.get("coriolis") == "rb" and same_controller and same_initial and same_initial_estimate and same_plant and same_timing and desired_samples)
    return {
        "exact_known_inertia": bool(exact_known),
        "payload_release": payload_release,
        "adaptation": adaptation,
        "lc_metadata_coriolis": lc_meta.get("coriolis"), "rb_metadata_coriolis": rb_meta.get("coriolis"),
        "shared_controller_gains": same_controller,
        "shared_plant_inertia": same_plant,
        "identical_initial_configuration": bool(np.array_equal(lc_run["H"][0], rb_run["H"][0])),
        "identical_initial_velocity": bool(np.array_equal(lc_run["V"][0], rb_run["V"][0])),
        "identical_desired_trajectory_samples": desired_samples,
        "only_intended_controller_difference": only_intended,
    }


def connection_pair_passed(protocol):
    return bool(
        protocol["exact_known_inertia"]
        and not protocol["payload_release"]
        and not protocol["adaptation"]
        and protocol["shared_controller_gains"]
        and protocol["shared_plant_inertia"]
        and protocol["identical_initial_configuration"]
        and protocol["identical_initial_velocity"]
        and protocol["identical_desired_trajectory_samples"]
        and protocol["lc_metadata_coriolis"] == "lc"
        and protocol["rb_metadata_coriolis"] == "rb"
        and protocol["only_intended_controller_difference"]
    )


def run_publication(command, duration, root, raw_root, reuse_cache=False):
    if command in ("all", "nominal-connection", "nominal-reaching", "connection-realizations", "connection-sensitivity"):
        if duration != 30. or reuse_cache:
            raise ValueError("The nominal sensitivity study requires fresh runs over source time 10–30 s")
        if command == "all":
            run_publication("adaptive-drop", duration, root, raw_root)
        from .connection_sensitivity import run_study
        return run_study(root, raw_root)
    root, raw_root = Path(root), Path(raw_root)
    root.mkdir(parents=True, exist_ok=True)
    report = {"optimization_run": False, "fair_adaptation_retuning_pending": True,
              "repeatability": "omitted: deterministic identical trials are not repeatability evidence"}
    if command != "all" and (root/"validation_status.json").exists():
        report.update(json.loads((root/"validation_status.json").read_text(encoding="utf-8")))
    if command in ("all", "adaptive-drop"):
        runs, scenarios, rows = {}, {}, []
        for mode in PRIMARY_MODES:
            scenario = paper_scenario(mode, duration)
            run, failure, cached = load_or_run(raw_root/"adaptive"/mode, scenario, reuse_cache)
            if cached:
                scenario = cached_scenario(run)
            print(f"{'Reusing' if cached else 'Running'} {NAMES[mode]}", flush=True)
            report[NAMES[mode]] = {"failure": failure, "end_time": float(run["t"][-1]) if len(run["t"]) else None}
            rows.append(adaptive_performance_row(run, mode, failure))
            if len(run["t"]): runs[mode], scenarios[mode] = run, scenario
        write_csv(root/"tables"/"adaptive_performance_summary.csv", rows)
        write_csv(root/"tables"/"baseline_comparison.csv", rows)
        write_csv(root/"tables"/"physical_consistency_summary.csv",
                  [physical_consistency_row(runs[mode], mode) for mode in ADAPTIVE_MODES if mode in runs])
        write_csv(root/"tables"/"controller_gain_summary.csv", controller_gain_rows(scenarios))
        gain = gain_report()
        gain["actual_saved_run_gains"] = controller_gain_rows(scenarios)[0]
        gain["known_inertia_baseline"] = "uses the true loaded inertia before release and true bare-vehicle inertia after release"
        write_json(root/"gain_summary.json", gain)
        if runs:
            adaptive_figures(runs, scenarios, root/"figures")
    else:
        write_json(root/"gain_summary.json", gain_report())
    repo = Path(__file__).resolve().parents[3]
    commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=repo, text=True).strip()
    source_files = sorted((repo/"src").rglob("*.py")) + [repo/"run"/"run_paper_sim_figures.py"]
    digest = hashlib.sha256()
    for path in source_files:
        digest.update(str(path.relative_to(repo)).replace("\\", "/").encode())
        digest.update(path.read_bytes())
    dirty = bool(subprocess.check_output(["git", "status", "--porcelain", "--untracked-files=normal"], cwd=repo, text=True).strip())
    write_json(root/"manifest.json", {"command": command, "duration_s": duration, "reuse_cache": reuse_cache, "base_commit": commit,
                                     "worktree_dirty_at_generation": dirty, "source_sha256": digest.hexdigest(),
                                     "environment": "agc", "report": report})
    write_json(root/"validation_status.json", report)
    (root/"diagnostic_report.md").write_text(DIAGNOSTICS + "\n## Current checks\n\n" + "\n".join(f"- {k}: {v}" for k, v in report.items()) + "\n", encoding="utf-8")
    (root/"metric_definitions.md").write_text(DEFINITIONS, encoding="utf-8")


DIAGNOSTICS = """# Numerical-results diagnostic report

## Corrected issues

- Connection residual had the wrong sign and used the reference twist instead of s; a nonzero-error regression test exposed it. Both wrenches now use the same sampled state/reference, including desired acceleration.
- Previous release peak was computed before release; previous post window started at 12 s. Windows now split exactly at 10 s.
- Attitude metrics are degrees, not unlabeled radians. Mixed force/torque effort is removed.
- Dwell checks formerly excluded the endpoint. Numerical reaching now uses the weighted s norm.
- Previous reaching experiment started on s=0 and had a vacuous zero bound. Isolated test starts with a nonzero twist and uses true inertia to recompute energy.
- Logged s and energy previously held stale controller samples; now evaluated at each plant sample. Estimates are logged before the next update. Physical margins are computed for both estimators.
- Tracking gains were independently tuned; all primary comparisons now share one saved optimized gain set.
- PyBullet previously recomputed inertia from collision geometry (Ixx approximately 0.0800 instead of 0.0409). URDF inertia loading, principal-inertia/body-frame transforms, body-origin wrench application and payload attachment frames are corrected and regression-tested. Existing optimized gains were obtained on that old plant and are not optimality evidence for the corrected plant.
- The relative connection residual is ill-conditioned near zero differences. Absolute residual is checked at EVERY sample with tolerance 1e-12+1e-10*signal norm; relative diagnostics exclude signal norms <=1e-6.

## Publication qualifications

- Connection-plot y-axis titles omit [1] for readability, but both norms are dimensionless: torque is divided by 1 N m and force by 1 N before taking the Euclidean norm. This is not mixed-unit wrench effort; see metric_definitions.md. The identity residual is theoretically zero at all times; its computed roundoff-level values remain on a logarithmic scale.

- Adaptation-only re-optimization is NOT run. Existing gamma and gamma_B are provisional, not a jointly fair optimized comparison. Do not claim optimality or estimator superiority from these runs.
- Historical optimizer records contain different tracking gains for Euclidean and Natural/Bregman variants. The published runs overwrite those common gains identically, but the records do not establish a common objective, trajectory, initial estimate, duration, constraints, and weights for gamma versus gamma_B.
- The Known-inertia controller receives the true loaded inertia before release and the true bare-vehicle inertia after release. It is included as a model-knowledge reference; the adaptive estimators do not receive this parameter switch. The separate nominal reaching test has no payload release.
- The matched LC/RB closed-loop study is a separate test from the same-state connection identity. Its protocol file records equal plant, initial state, reference samples, and gains; only the Coriolis realization changes. Small off-manifold differences are expected and neither realization is ranked.
- No rotor allocation or actuator limits exist in this ideal wrench-actuated model. These plots do not establish hardware feasibility.
- Payload release violates the constant-parameter assumption at the jump. Adaptive asymptotic theory does not assert finite-time reaching or parameter convergence.
- Bregman stepping uses an SPD-preserving exponential update with numerical eigenvalue/exponent safeguards; it is not exact continuous-time integration.
- Identical repeated trials and all old flat figures/tables are superseded and must not be cited.
- FAILED reaching figures are diagnostics only; numerical threshold crossing is not exact finite-time convergence.
- All payload time histories use 0--30 s. Every figure in 04-nominal-validation uses the 4x connection sensitivity experiment on source time 10--30 s. Reaching, identity, and sensitivity-detail figures use the shared observed-reaching window; summaries retain elapsed reaching durations. The connection residual remains logarithmic. See connection_sensitivity_diagnostics.md for the experiment and numerical limitations.
- The nominal transverse-energy integral's relative numerical residual is retained in the reaching summary. Report the threshold-and-dwell bound check as numerical evidence, not as a pointwise reproduction of the continuous-time energy identity.
"""

DEFINITIONS = """# Metric definitions

Uniform plant samples include t=0 and the final sample. Pre-release means t<10 s;
post-release means t>=10 s. RMSE = sqrt(mean(error_norm**2)) over that window.
Position norm uses inertial positions in metres. Attitude error is
acos(clip((trace(Rd.T R)-1)/2,-1,1)), converted to degrees. Peaks are window maxima.
No transient is removed. Full-run force/torque RMS = sqrt(mean(norm**2)); peaks
are max(norm), separately in N and N m. Commands are the actual zero-order-held wrenches.

Recovery: first sampled t>=10 for which position<=0.05 m AND geodesic angle<=5 deg
at every sample through the first sample at or after t+1 s (inclusive).
Report absolute time and duration t-10. Blank means not observed with a complete dwell.
This is sampled dwell evidence, not a guarantee between samples or for all future time.

Observed reaching: first sample with sqrt(s.T Lambda_s s)<=0.001 throughout a
0.5 s sampled dwell, including the endpoint. No incomplete terminal dwell qualifies.
Lambda_s=inverse(Lambda). Bound uses true I and actual s(0), never estimated energy.
The nominal controller is continuous in theory but evaluated at finite sample rate.

Physical margin: smallest eigenvalue of Jhat directly (Bregman) or pseudo_from_pi
(Euclidean), with full-run and post-release minima and nonpositive flag.
Pseudo-inertia combines kg, kg m, kg m^2; eigenvalues are coordinate-scaled SI
certificates, not a scalar with one physical unit. Likewise the weighted s norm
uses the specified design metric and therefore has no single physical unit.

RPY is xyz Euler visualization, unwrapped independently in time and aligned by
integer 360-degree offsets initially; geodesic error is used for all quantitative claims.
Desired velocity in each actual body is Ad_(He^-1) Vd, including the translational
adjoint term. These transported references differ slightly between controllers;
the black dashed curves are labelled "Transported reference" and show each,
not an incorrect shared raw Vd.

## Connection-equivalence norm and axis labels

For a wrench w = [tau; f], the dimensionless scaled norm is

```text
||w||_* = sqrt(sum_i (tau_i / (1 N m))^2 + sum_i (f_i / (1 N))^2).
```

The same scaling applies to the wrench difference, K_RB(V)s, and the identity
residual r_K. With unit reference scales in SI, this equals the Euclidean norm
of the stored six numerical components. It is a visualization/validation norm,
not an energy or control-effort measure. Other reference scales change the
magnitude but not whether the identity residual is zero.

Both y-axes are dimensionless. Their titles are "Scaled wrench norm" and
"Scaled residual norm"; [1] is intentionally omitted for readability, not
replaced by N or N m. Force and torque metrics elsewhere retain separate units.

Theory predicts r_K = W_RB - W_LC + K_RB(V)s = 0 at every time, including off
the sliding manifold. The wrench difference itself need only vanish when s=0.
The computed small residual is consistent with floating-point roundoff; the
logarithmic panel retains these values rather than setting them to zero.
T_obs marks numerical threshold-and-dwell reaching, not the onset of validity
of the algebraic identity.

Relative residual uses max(norm(left),norm(right)) only above 1e-6.
Separate maximum force and torque residuals are also saved.

## Center-of-mass and principal-inertia diagnostics

The additional estimator figures reconstruct body-frame c = h/m [m] and
J_c = J_b - m ((c.T c) I - c c.T) [kg m^2] from estimatePi.
True references use activePlantPi at the same logged samples, including the
payload jump at 10 s. Principal moments are the ascending eigenvalues of J_c,
not body-origin diagonal entries or eigenvalues of the pseudo-inertia.
Their indices denote magnitude ordering, not continuously tracked principal axes.

Nonfinite parameter samples, nonpositive mass, or nonfinite reconstructions
produce warnings and gaps. Finite negative principal moments remain visible;
there is no filtering or projection to physical values. These are estimator
trajectories, not evidence of parameter convergence or sufficient excitation.
The existing pseudo-inertia certificate remains the physical-consistency test.

Exports: estimated_center_of_mass.pdf/png and estimated_principal_inertia.pdf/png
in figures/02-physical-consistency. Both show 0–30 s with a 10 s release marker,
Euclidean blue dash-dot, Natural/Bregman solid green, and true values black dashed.
"""
