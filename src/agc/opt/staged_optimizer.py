"""Hierarchical and classic staged block-coordinate gain optimization runner."""

import os
import json
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, Any, List, Optional, Tuple
import numpy as np

from .bounds import (
    gain_bounds,
    gain_block_indices,
    gain_optimization_stages,
    expand_scenario_selection,
)
from .encoding import (
    encode_scenario_gains,
    apply_scenario_gains,
    apply_gain_block,
    round_gains,
)
from .objective import (
    objective_weights,
    evaluate_scenario_set_candidate,
    best_feasible_candidate,
)
from .pso import ParticleSwarmOptimizer
from .bregman_profile import profile_bregman_gain
from ..sim.default_scenario import default_scenario, get_repository_root
from ..io.persistence import default_results_root, load_best_gain, save_best_gain


DEFAULT_TRAINING_REPLAY_IDS = ("lemniscate_02_auto", "lemniscate_03_auto")
DEFAULT_TRAINING_PAYLOAD_PROFILES = ("flat_light", "tall_heavy")
DEFAULT_TRAINING_CORIOLIS_FORMS = ("lc", "rb")


def default_training_conditions(
    replay_ids: Optional[List[str]] = None,
    payload_profiles: Optional[List[str]] = None,
    coriolis_forms: Optional[List[str]] = None,
) -> List[Dict[str, str]]:
    """Return the Cartesian product of replay, payload, and Coriolis conditions for gain tuning."""
    selected_replays = list(DEFAULT_TRAINING_REPLAY_IDS) if replay_ids is None else replay_ids
    selected_profiles = list(DEFAULT_TRAINING_PAYLOAD_PROFILES) if payload_profiles is None else payload_profiles
    selected_coriolis = list(DEFAULT_TRAINING_CORIOLIS_FORMS) if coriolis_forms is None else coriolis_forms

    if not selected_replays or not selected_profiles or not selected_coriolis:
        raise ValueError("Training requires at least one replay ID, payload profile, and coriolis form.")

    return [
        {"replayId": str(replay_id), "payloadProfile": str(profile), "coriolis": str(coriolis)}
        for replay_id in selected_replays
        for profile in selected_profiles
        for coriolis in selected_coriolis
    ]


def is_strictly_improved(candidate: Dict[str, Any], reference: Dict[str, Any]) -> bool:
    """Return True if candidate is feasible and strictly lower cost than reference."""
    cand_valid = (not candidate.get("failed", True)) and np.isfinite(candidate.get("cost", float("inf")))
    ref_valid = (not reference.get("failed", True)) and np.isfinite(reference.get("cost", float("inf")))
    if not cand_valid:
        return False
    if not ref_valid:
        return True
    return float(candidate["cost"]) < float(reference["cost"])


class BlockCostEvaluator:
    """Picklable top-level worker for evaluating block-coordinate candidates."""

    def __init__(self, incumbent_candidate: np.ndarray, block: str, scenarios: List[Dict[str, Any]], weights: Dict[str, float]):
        self.incumbent_candidate = np.asarray(incumbent_candidate, dtype=float)
        self.block = block
        self.scenarios = scenarios
        self.weights = weights

    def __call__(self, block_x: np.ndarray) -> float:
        sc = apply_gain_block(block_x, self.incumbent_candidate, self.block, self.scenarios[0])
        rec = evaluate_scenario_set_candidate(encode_scenario_gains(sc), self.scenarios, self.weights, label=self.block)
        return float(rec["cost"])


def promote_gains_to_registry(
    mode: str,
    coriolis: Optional[str] = "all",
    gains: Optional[Dict[str, Any]] = None,
    metadata: Optional[Dict[str, Any]] = None,
    target_file: Optional[str] = None,
):
    """Update the Python optimized-gains registry with improved gains."""
    root = get_repository_root()
    if target_file is None:
        target_path = root / "src" / "agc" / "config" / "optimized_gains.py"
    else:
        target_path = Path(target_file)

    import sys
    import importlib
    if "agc.config.optimized_gains" in sys.modules:
        importlib.reload(sys.modules["agc.config.optimized_gains"])
    from ..config.optimized_gains import optimized_gains as get_current_gains

    modes = ["nominal", "euclidean", "bregman"]
    forms = ["lc", "rb"]

    registry = {}
    for m in modes:
        for f in forms:
            key = f"{m}_{f}"
            registry[key] = get_current_gains(m, f)

    updated = round_gains(gains, sig_figs=4) if gains is not None else {}
    if metadata is not None and metadata.get("cost") is not None:
        updated["optimizationCost"] = float(metadata["cost"])

    target_mode = mode.lower()
    tracking_keys = ["KRdiag", "Kxidiag", "LambdaDiag", "kd", "ks", "alpha"]

    if target_mode == "nominal":
        # Synchronize tracking gains across all controllers and Coriolis forms
        for m in modes:
            for f in forms:
                k = f"{m}_{f}"
                for tk in tracking_keys:
                    if tk in updated:
                        registry[k][tk] = updated[tk]
                if m == "nominal":
                    registry[k]["optimizationCost"] = updated.get("optimizationCost")
    elif target_mode in ("euclidean", "adaptive"):
        for f in forms:
            k = f"euclidean_{f}"
            if "gammaE" in updated:
                registry[k]["gammaE"] = updated["gammaE"]
            registry[k]["optimizationCost"] = updated.get("optimizationCost")
    elif target_mode == "bregman":
        for f in forms:
            k = f"bregman_{f}"
            if "gammaB" in updated:
                registry[k]["gammaB"] = updated["gammaB"]
            registry[k]["optimizationCost"] = updated.get("optimizationCost")

    # Mirror adaptive and mode-level aliases
    for f in forms:
        registry[f"adaptive_{f}"] = dict(registry[f"euclidean_{f}"])
    registry["adaptive"] = dict(registry["euclidean_lc"])
    registry["nominal"] = dict(registry["nominal_lc"])
    registry["euclidean"] = dict(registry["euclidean_lc"])
    registry["bregman"] = dict(registry["bregman_lc"])

    # Write formatted python file
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    stage_str = metadata.get("stage", "N/A") if metadata else "N/A"
    cost_val = metadata.get("cost", None) if metadata else None
    cost_str = f"{float(cost_val):.6g}" if cost_val is not None else "N/A"

    lines = [
        '"""Optimized per-scenario gain registry.',
        f"Last promotion: {timestamp} (Stage: {stage_str}, Cost: {cost_str})",
        '"""',
        "",
        "import numpy as np",
        "",
        "",
        'def optimized_gains(mode: str, coriolis: str = "lc") -> dict:',
        '    """Return tuned gains for each controller variant (invariant to Coriolis)."""',
        "    m = mode.lower()",
        '    c = coriolis.lower() if coriolis else "lc"',
        '    key = f"{m}_{c}"',
        "    registry = {",
    ]

    for k, v in registry.items():
        lines.append(f'        "{k}": {{')
        lines.append(f'            "KRdiag": np.array({list(np.round(v["KRdiag"], 6))}),')
        lines.append(f'            "Kxidiag": np.array({list(np.round(v["Kxidiag"], 6))}),')
        lines.append(f'            "LambdaDiag": np.array({list(np.round(v["LambdaDiag"], 6))}),')
        lines.append(f'            "kd": {v["kd"]:.6g},')
        lines.append(f'            "ks": {v["ks"]:.6g},')
        lines.append(f'            "alpha": {v["alpha"]:.6g},')
        if v.get("gammaE") is not None:
            ge_list = list(np.round(np.asarray(v["gammaE"], dtype=float).ravel(), 6))
            lines.append(f'            "gammaE": np.array({ge_list}),')
        else:
            lines.append('            "gammaE": 0.001 * np.ones(10),')
        if v.get("gammaB") is not None:
            lines.append(f'            "gammaB": {float(v["gammaB"]):.6g},')
        else:
            lines.append('            "gammaB": 0.001,')
        cost = v.get("optimizationCost")
        if cost is None:
            lines.append('            "optimizationCost": None,')
        else:
            lines.append(f'            "optimizationCost": {float(cost):.12g},')
        lines.append("        },")

    lines.extend([
        "    }",
        "    if key in registry:",
        "        return registry[key]",
        "    if m in registry:",
        "        return registry[m]",
        '    raise ValueError(f"Unknown mode/coriolis combination: {mode}/{coriolis}")',
        "",
    ])

    target_path.write_text("\n".join(lines), encoding="utf-8")


def _run_optimization_stage_loop(
    stage_names: List[str],
    sel_mode: str,
    incumbent: Dict[str, Any],
    manual_rec: Dict[str, Any],
    reg_rec: Dict[str, Any],
    manual_scenarios: List[Dict[str, Any]],
    weights: Dict[str, float],
    swarm_size: int,
    max_iter: int,
    max_stall: int,
    tol: float,
    parallel: bool,
    seed: Optional[int],
    res_dir: Path,
    promote: bool,
    best_gain_path: Path,
    training_conditions: List[Dict[str, str]],
    schedule: str,
    extra_seeds: Optional[List[np.ndarray]] = None,
) -> Tuple[Dict[str, Any], List[Dict[str, Any]]]:
    """Execute block optimization stages using ParticleSwarmOptimizer."""
    lb_full, ub_full = gain_bounds(sel_mode)
    stages_log = []
    promoted = incumbent

    for s_idx, s_name in enumerate(stage_names, start=1):
        print(f"\n  --- Stage {s_idx}/{len(stage_names)}: {s_name} ---")

        seeds_full = [manual_rec["candidate"], reg_rec["candidate"], incumbent["candidate"]]
        if extra_seeds:
            for es in extra_seeds:
                if es is not None:
                    seeds_full.append(np.asarray(es, dtype=float).ravel())
        unique_seeds_full = np.unique(np.array(seeds_full), axis=0)

        if s_name == "adaptive" and sel_mode == "bregman":
            # 1D log-grid scan for scalar gammaB
            seed_vals = 10.0 ** unique_seeds_full[:, 15]
            profile = profile_bregman_gain(
                base_scenarios=manual_scenarios,
                incumbent_candidate=incumbent["candidate"],
                seed_values=seed_vals.tolist(),
                weights=weights,
                parallel=parallel,
            )
            stage_result = {"method": "log-grid", "profile": profile}
            contender = profile["best"]
        else:
            indices = gain_block_indices(sel_mode, s_name)
            lb_block = lb_full[indices]
            ub_block = ub_full[indices]
            block_seeds = np.unique(unique_seeds_full[:, indices], axis=0)

            block_cost = BlockCostEvaluator(incumbent["candidate"], s_name, manual_scenarios, weights)

            opt = ParticleSwarmOptimizer(
                cost_func=block_cost,
                lower_bound=lb_block,
                upper_bound=ub_block,
                swarm_size=swarm_size,
                max_iter=max_iter,
                max_stall=max_stall,
                tol=tol,
                initial_points=block_seeds,
                parallel=parallel,
                verbose=True,
                seed=seed,
            )

            best_block_x, best_cost, history = opt.optimize()

            full_cand = np.copy(incumbent["candidate"])
            full_cand[indices] = best_block_x
            contender = evaluate_scenario_set_candidate(full_cand, manual_scenarios, weights, label=s_name)
            stage_result = {
                "method": "pso",
                "block": s_name,
                "indices": indices,
                "history": history,
                "best_block_x": best_block_x.tolist(),
            }

        prev_incumbent_cost = incumbent["cost"]
        incumbent = best_feasible_candidate(incumbent, contender)
        print(f"    Contender cost:  {contender['cost']:.6g} (failed={contender['failed']})")
        print(f"    Incumbent cost:  {incumbent['cost']:.6g} ({incumbent['label']})")

        stage_entry = {
            "name": s_name,
            "contender_cost": float(contender["cost"]),
            "incumbent_cost": float(incumbent["cost"]),
            "improved": float(incumbent["cost"]) < prev_incumbent_cost,
            "result": stage_result,
        }
        stages_log.append(stage_entry)

        chkpt = {
            "mode": sel_mode,
            "schedule": schedule,
            "trainingConditions": training_conditions,
            "stage_index": s_idx,
            "stages": stages_log,
            "incumbent_cost": float(incumbent["cost"]),
            "incumbent_label": incumbent["label"],
        }
        with open(res_dir / "optimization.json", "w", encoding="utf-8") as f:
            json.dump(
                chkpt,
                f,
                indent=2,
                default=lambda o: o.tolist() if isinstance(o, np.ndarray) else str(o),
            )

        if promote and is_strictly_improved(incumbent, promoted):
            print(f"    >>> PROMOTING NEW BEST GAINS for {sel_mode} (Cost: {incumbent['cost']:.6g} < {promoted['cost']:.6g})")
            promote_gains_to_registry(
                mode=sel_mode,
                coriolis="all",
                gains=incumbent["gains"],
                metadata={"cost": incumbent["cost"], "stage": s_name, "artifact": str(res_dir)},
            )
            save_best_gain(
                best_gain_path, sel_mode, "all", incumbent["gains"],
                incumbent["cost"], s_name, incumbent["candidate"],
            )
            promoted = incumbent

    return incumbent, stages_log


def run_staged_optimization(
    mode: str = "all",
    coriolis: Optional[str] = "all",
    schedule: str = "hierarchical",
    duration: float = 30.0,
    replay_id: Optional[str] = None,
    training_replay_ids: Optional[List[str]] = None,
    training_payload_profiles: Optional[List[str]] = None,
    swarm_size: int = 50,
    max_iter: int = 20,
    max_stall: int = 10,
    tol: float = 1e-3,
    parallel: bool = True,
    promote: bool = True,
    output_dir: Optional[str] = None,
    seed: Optional[int] = None,
    inplace_save: bool = True,
) -> Dict[str, Any]:
    """Execute staged block-coordinate optimization for selected scenarios."""
    weights = objective_weights()
    root = get_repository_root()
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    if replay_id is not None and training_replay_ids is None:
        training_replay_ids = [replay_id]

    coriolis_forms = (
        list(DEFAULT_TRAINING_CORIOLIS_FORMS)
        if (coriolis is None or str(coriolis).lower() == "all")
        else [str(coriolis).lower()]
    )

    training_conditions = default_training_conditions(
        training_replay_ids, training_payload_profiles, coriolis_forms
    )

    modes_to_run = expand_scenario_selection(mode)
    overall_results = {}

    for var_idx, sel_mode in enumerate(modes_to_run, start=1):
        print("=" * 70)
        print(f"Optimizing Mode: {sel_mode.upper()} ({var_idx} of {len(modes_to_run)})")
        print(f"Schedule: {schedule} | Method: PSO | Parallel: {parallel}")
        print(f"Training conditions: {len(training_conditions)} (Coriolis: {coriolis_forms})")
        print("=" * 70)

        result_root = default_results_root(str(root), inplace_save=inplace_save, timestamp=stamp)
        res_dir = (
            Path(output_dir) / sel_mode
            if output_dir is not None
            else result_root / sel_mode
        )
        os.makedirs(res_dir, exist_ok=True)

        if sel_mode == "nominal":
            # 1. Nominal controller optimization across all 8 conditions
            manual_scenarios = [
                default_scenario(
                    replay_id=c["replayId"], mode="nominal", coriolis=c["coriolis"],
                    duration=duration, gain_source="manual", gui=False, enable_pacing=False,
                    payload_profile=c["payloadProfile"],
                )
                for c in training_conditions
            ]
            reg_scenarios = [
                default_scenario(
                    replay_id=c["replayId"], mode="nominal", coriolis=c["coriolis"],
                    duration=duration, gain_source="optimized", gui=False, enable_pacing=False,
                    payload_profile=c["payloadProfile"],
                )
                for c in training_conditions
            ]

            manual_cand = encode_scenario_gains(manual_scenarios[0])
            reg_cand = encode_scenario_gains(reg_scenarios[0])
            manual_rec = evaluate_scenario_set_candidate(manual_cand, manual_scenarios, weights, label="manual")
            reg_rec = evaluate_scenario_set_candidate(reg_cand, manual_scenarios, weights, label="registered")
            incumbent = best_feasible_candidate(manual_rec, reg_rec)

            # Known robust historical baseline candidate as an additional high-quality seed
            hist_gains = {
                "KRdiag": np.array([8.737, 12.15, 2.74]),
                "Kxidiag": np.array([67.45, 97.26, 92.51]),
                "LambdaDiag": np.array([6.979, 6.767, 4.755, 0.1, 0.1278, 0.1136]),
                "kd": 0.5913,
                "ks": 22.97,
                "alpha": 0.8929,
            }
            hist_pos = np.concatenate([hist_gains["KRdiag"], hist_gains["Kxidiag"], hist_gains["LambdaDiag"], [hist_gains["kd"], hist_gains["ks"]]])
            hist_cand = np.array(list(np.log10(hist_pos)) + [hist_gains["alpha"]], dtype=float)
            hist_rec = evaluate_scenario_set_candidate(hist_cand, manual_scenarios, weights, label="historical-robust")
            incumbent = best_feasible_candidate(incumbent, hist_rec)

            best_gain_path = result_root / "nominal.json"
            saved_best = load_best_gain(best_gain_path)
            if saved_best is not None and "candidate" in saved_best:
                saved_cand = np.asarray(saved_best["candidate"], dtype=float)
                saved_rec = evaluate_scenario_set_candidate(saved_cand, manual_scenarios, weights, label="persisted-best")
                incumbent = best_feasible_candidate(incumbent, saved_rec)

            stage_names = gain_optimization_stages("nominal", schedule=schedule)
            best_cand, stages_log = _run_optimization_stage_loop(
                stage_names=stage_names,
                sel_mode="nominal",
                incumbent=incumbent,
                manual_rec=manual_rec,
                reg_rec=reg_rec,
                manual_scenarios=manual_scenarios,
                weights=weights,
                swarm_size=swarm_size,
                max_iter=max_iter,
                max_stall=max_stall,
                tol=tol,
                parallel=parallel,
                seed=seed,
                res_dir=res_dir,
                promote=promote,
                best_gain_path=best_gain_path,
                training_conditions=training_conditions,
                schedule=schedule,
                extra_seeds=[hist_rec["candidate"]],
            )
            overall_results["nominal"] = {
                "incumbent": best_cand,
                "artifact_dir": str(res_dir),
                "stages": stages_log,
            }

        elif sel_mode == "adaptive":
            # 2. Adaptive optimization:
            # First: Stages 1-5 tune/refine common tracking gains on nominal baseline
            # Then: Stage 'adaptive' for Euclidean (gammaE) and Bregman (gammaB)
            # Final 'all' is omitted to avoid bias.
            print("  --- Phase 1: Shared Base Tracking Gains ---")
            nominal_scenarios = [
                default_scenario(
                    replay_id=c["replayId"], mode="nominal", coriolis=c["coriolis"],
                    duration=duration, gain_source="optimized", gui=False, enable_pacing=False,
                    payload_profile=c["payloadProfile"],
                )
                for c in training_conditions
            ]
            manual_scenarios = [
                default_scenario(
                    replay_id=c["replayId"], mode="nominal", coriolis=c["coriolis"],
                    duration=duration, gain_source="manual", gui=False, enable_pacing=False,
                    payload_profile=c["payloadProfile"],
                )
                for c in training_conditions
            ]

            nom_cand = encode_scenario_gains(nominal_scenarios[0])
            man_cand = encode_scenario_gains(manual_scenarios[0])
            nom_rec = evaluate_scenario_set_candidate(nom_cand, nominal_scenarios, weights, label="registered-nominal")
            man_rec = evaluate_scenario_set_candidate(man_cand, nominal_scenarios, weights, label="manual-nominal")
            incumbent_base = best_feasible_candidate(man_rec, nom_rec)

            # Known robust historical baseline candidate as an additional high-quality seed
            hist_gains = {
                "KRdiag": np.array([8.737, 12.15, 2.74]),
                "Kxidiag": np.array([67.45, 97.26, 92.51]),
                "LambdaDiag": np.array([6.979, 6.767, 4.755, 0.1, 0.1278, 0.1136]),
                "kd": 0.5913,
                "ks": 22.97,
                "alpha": 0.8929,
            }
            hist_pos = np.concatenate([hist_gains["KRdiag"], hist_gains["Kxidiag"], hist_gains["LambdaDiag"], [hist_gains["kd"], hist_gains["ks"]]])
            hist_cand = np.array(list(np.log10(hist_pos)) + [hist_gains["alpha"]], dtype=float)
            hist_rec = evaluate_scenario_set_candidate(hist_cand, nominal_scenarios, weights, label="historical-robust")
            incumbent_base = best_feasible_candidate(incumbent_base, hist_rec)

            best_nom_path = result_root / "nominal.json"
            saved_best = load_best_gain(best_nom_path)
            if saved_best is not None and "candidate" in saved_best:
                saved_cand = np.asarray(saved_best["candidate"], dtype=float)
                saved_rec = evaluate_scenario_set_candidate(saved_cand, nominal_scenarios, weights, label="persisted-best")
                incumbent_base = best_feasible_candidate(incumbent_base, saved_rec)

            # Stages 1 to 5 of the hierarchical schedule (all through dissipation)
            base_stages = ["all", "tracking", "sliding_dissipation", "sliding_metric", "dissipation"]
            if schedule == "classic":
                base_stages = ["all"]

            best_base, base_stages_log = _run_optimization_stage_loop(
                stage_names=base_stages,
                sel_mode="nominal",
                incumbent=incumbent_base,
                manual_rec=man_rec,
                reg_rec=nom_rec,
                manual_scenarios=nominal_scenarios,
                weights=weights,
                swarm_size=swarm_size,
                max_iter=max_iter,
                max_stall=max_stall,
                tol=tol,
                parallel=parallel,
                seed=seed,
                res_dir=res_dir,
                promote=promote,
                best_gain_path=best_nom_path,
                training_conditions=training_conditions,
                schedule=schedule,
                extra_seeds=[hist_rec["candidate"]],
            )

            # Common tracking gains established; freeze them for adaptive estimators
            common_base_candidate = np.copy(best_base["candidate"])
            adaptive_results = {}

            # Tune each adaptive estimator separately on stage 'adaptive' only
            for adapt_type in ["euclidean", "bregman"]:
                print(f"\n  --- Phase 2: Tuning {adapt_type.upper()} Adaptation Gains ---")
                # Bregman affine-invariant adaptation preserves passivity under skew-symmetric Coriolis (LC).
                # To prevent non-skew-symmetric perturbations (RB) from suppressing adaptation toward zero,
                # tune Bregman adaptation on LC conditions, while tracking gains remain Coriolis-invariant.
                conds = (
                    [c for c in training_conditions if c["coriolis"] == "lc"]
                    if adapt_type == "bregman"
                    else training_conditions
                )
                adapt_scenarios = [
                    default_scenario(
                        replay_id=c["replayId"], mode=adapt_type, coriolis=c["coriolis"],
                        duration=duration, gain_source="optimized", gui=False, enable_pacing=False,
                        payload_profile=c["payloadProfile"],
                    )
                    for c in conds
                ]

                # Assemble candidate with locked base tracking gains + current adaptive seed
                reg_adapt_cand = encode_scenario_gains(adapt_scenarios[0])
                locked_cand = np.copy(reg_adapt_cand)
                locked_cand[0:15] = common_base_candidate[0:15]

                adapt_rec = evaluate_scenario_set_candidate(locked_cand, adapt_scenarios, weights, label=f"seed-{adapt_type}")
                adapt_best_path = result_root / f"{adapt_type}.json"
                adapt_saved = load_best_gain(adapt_best_path)
                if adapt_saved is not None and "candidate" in adapt_saved:
                    cand_saved = np.asarray(adapt_saved["candidate"], dtype=float)
                    cand_saved[0:15] = common_base_candidate[0:15]
                    saved_rec = evaluate_scenario_set_candidate(cand_saved, adapt_scenarios, weights, label="persisted-adapt")
                    adapt_rec = best_feasible_candidate(adapt_rec, saved_rec)

                adapt_res_dir = res_dir / adapt_type
                os.makedirs(adapt_res_dir, exist_ok=True)

                best_adapt, adapt_stages_log = _run_optimization_stage_loop(
                    stage_names=["adaptive"],
                    sel_mode=adapt_type,
                    incumbent=adapt_rec,
                    manual_rec=adapt_rec,
                    reg_rec=adapt_rec,
                    manual_scenarios=adapt_scenarios,
                    weights=weights,
                    swarm_size=swarm_size,
                    max_iter=max_iter,
                    max_stall=max_stall,
                    tol=tol,
                    parallel=parallel,
                    seed=seed,
                    res_dir=adapt_res_dir,
                    promote=promote,
                    best_gain_path=adapt_best_path,
                    training_conditions=conds,
                    schedule=schedule,
                )
                adaptive_results[adapt_type] = {
                    "incumbent": best_adapt,
                    "artifact_dir": str(adapt_res_dir),
                    "stages": adapt_stages_log,
                }

            overall_results["adaptive"] = {
                "common_tracking": best_base,
                "estimators": adaptive_results,
                "base_stages": base_stages_log,
            }

    return overall_results
