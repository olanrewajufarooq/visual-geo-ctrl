"""Run the publication protocol without optimization or deterministic repeats."""
from pathlib import Path
import hashlib
import json
import subprocess
import numpy as np
from .publication import paper_scenario, gain_report, baseline_metrics, write_json, write_csv, reaching_summary, NAMES, ADAPTIVE_MODES
from .default_scenario import default_scenario
from .run_scenario import run_scenario
from ..io.persistence import save_run
from ..paper.diagnostics import connection_identity
from ..math.inertia import inertia_from_pi
from ..viz.publication_figures import adaptive_figures, theory_figures


def nominal_scenario(duration, dt=.002):
    scenario = default_scenario(mode="nominal", payload_enabled=False, duration=duration, enable_pacing=False)
    desired = {"H": np.eye(4), "V": np.zeros(6), "Vdot": np.zeros(6)}
    desired["H"][2, 3] = 2.
    scenario["trajectory"] = lambda t: {k: v.copy() for k, v in desired.items()}
    scenario["replayId"] = "fixed-reference-nonzero-initial-twist"
    scenario["initial"] = {"H": desired["H"].copy(), "V": np.array([.3, -.2, .1, .4, -.2, .3])}
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


def run_publication(command, duration, root, raw_root):
    root, raw_root = Path(root), Path(raw_root)
    root.mkdir(parents=True, exist_ok=True)
    report = {"optimization_run": False, "fair_adaptation_retuning_pending": True,
              "repeatability": "omitted: deterministic identical trials are not repeatability evidence"}
    if command != "all" and (root/"validation_status.json").exists():
        report.update(json.loads((root/"validation_status.json").read_text(encoding="utf-8")))
    report.pop(NAMES["nominal"], None)
    write_json(root/"gain_summary.json", gain_report())
    if command in ("all", "adaptive-drop"):
        runs, scenarios, rows = {}, {}, []
        for mode in ADAPTIVE_MODES:
            print(f"Running {NAMES[mode]} with shared tracking gains", flush=True)
            scenario = paper_scenario(mode, duration)
            run, failure = run_scenario(scenario)
            save_run(str(raw_root/"adaptive"/mode), run, None, scenario, failure)
            report[NAMES[mode]] = {"failure": failure, "end_time": float(run["t"][-1]) if len(run["t"]) else None}
            rows.append(baseline_metrics(run, mode, failure))
            if len(run["t"]): runs[mode], scenarios[mode] = run, scenario
        write_csv(root/"tables"/"baseline_comparison.csv", rows)
        if runs:
            adaptive_figures(runs, scenarios, root/"figures")
    if command in ("all", "nominal-connection", "nominal-reaching"):
        print("Running isolated known-inertia validation", flush=True)
        scenario = nominal_scenario(duration)
        run, failure = run_scenario(scenario)
        save_run(str(raw_root/"nominal"), run, None, scenario, failure)
        summary = reaching_summary(run, scenario)
        summary["failure"] = failure
        # Integration refinement is validation, not gain optimization.
        refined_scenario = nominal_scenario(min(duration, 4.), dt=.001)
        refined_run, refined_failure = run_scenario(refined_scenario)
        save_run(str(raw_root/"nominal-refinement"), refined_run, None, refined_scenario, refined_failure)
        refined = reaching_summary(refined_run, refined_scenario)
        summary["step_refinement"] = {"dt_coarse_s": .002, "dt_fine_s": .001,
            "T_obs_fine": refined["T_obs"], "fine_passed": refined["passed"], "failure": refined_failure,
            "fine_energy_residual_relative_to_initial": refined["energy_residual_relative_to_initial"]}
        connection = connection_test(run, scenario)
        write_json(root/"finite_time_reaching_summary.json", summary)
        write_json(root/"connection_equivalence_summary.json", {k: v for k, v in connection.items() if k not in ("t", "difference", "theory", "residual")})
        theory_figures(run, scenario, root/"figures", summary, connection)
        report["nominal_reaching_passed"] = summary["passed"]
        report["connection_identity_passed"] = connection["passed"]
        report["nominal_refinement_passed"] = refined["passed"]
        if not summary["passed"]:
            print("Reaching validation FAILED; retaining diagnostic figure, not a paper success claim.", flush=True)
    repo = Path(__file__).resolve().parents[3]
    commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=repo, text=True).strip()
    source_files = sorted((repo/"src").rglob("*.py")) + [repo/"run"/"run_paper_experiments.py"]
    digest = hashlib.sha256()
    for path in source_files:
        digest.update(str(path.relative_to(repo)).replace("\\", "/").encode())
        digest.update(path.read_bytes())
    dirty = bool(subprocess.check_output(["git", "status", "--porcelain", "--untracked-files=normal"], cwd=repo, text=True).strip())
    write_json(root/"manifest.json", {"command": command, "duration_s": duration, "base_commit": commit,
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
- The fixed-model controller is excluded from adaptive figures, metrics and default runs. The separate nominal theory tests use exact known inertia, not an adaptive estimator.
- No rotor allocation or actuator limits exist in this ideal wrench-actuated model. These plots do not establish hardware feasibility.
- Payload release violates the constant-parameter assumption at the jump. Adaptive asymptotic theory does not assert finite-time reaching or parameter convergence.
- Bregman stepping uses an SPD-preserving exponential update with numerical eigenvalue/exponent safeguards; it is not exact continuous-time integration.
- Identical repeated trials and all old flat figures/tables are superseded and must not be cited.
- FAILED reaching figures are diagnostics only; numerical threshold crossing is not exact finite-time convergence.
- All payload time histories use 0--30 s. Isolated reaching and connection-equivalence figures share a focused reaching-interval view. The connection residual remains logarithmic and its x-axis measures time since initialization. Nominal runs have no payload-release marker because no release occurs.
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
uses the specified design metric and is labelled metric units.

RPY is xyz Euler visualization, unwrapped independently in time and aligned by
integer 360-degree offsets initially; geodesic error is used for all quantitative claims.
Desired velocity in each actual body is Ad_(He^-1) Vd, including the translational
adjoint term. These transported references differ slightly between controllers;
the black dashed curves show each, not an incorrect shared raw Vd.

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
"""
