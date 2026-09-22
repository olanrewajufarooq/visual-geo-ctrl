"""Hierarchical and classic staged block-coordinate gain optimization runner."""

import os
import json
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, Any, List, Optional
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
from .de import DifferentialEvolutionOptimizer, nelder_mead_polish
from .bregman_profile import profile_bregman_gain
from ..sim.default_scenario import default_scenario, get_repository_root
from ..io.persistence import default_results_root, load_best_gain, save_best_gain


DEFAULT_TRAINING_REPLAY_IDS = ("lemniscate_02_auto", "lemniscate_03_auto", "lemniscate_04_auto")
DEFAULT_TRAINING_PAYLOAD_PROFILES = ("flat_light", "tall_heavy")


def default_training_conditions(
    replay_ids: Optional[List[str]] = None,
    payload_profiles: Optional[List[str]] = None,
) -> List[Dict[str, str]]:
    """Return the Cartesian product of replay and payload conditions for gain tuning."""
    selected_replays = list(DEFAULT_TRAINING_REPLAY_IDS) if replay_ids is None else replay_ids
    selected_profiles = list(DEFAULT_TRAINING_PAYLOAD_PROFILES) if payload_profiles is None else payload_profiles
    if not selected_replays or not selected_profiles:
        raise ValueError("Training requires at least one replay ID and payload profile.")
    return [
        {"replayId": str(replay_id), "payloadProfile": str(profile)}
        for replay_id in selected_replays
        for profile in selected_profiles
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
    coriolis: str,
    gains: Dict[str, Any],
    metadata: Optional[Dict[str, Any]] = None,
    target_file: Optional[str] = None,
):
    """Update the Python optimized-gains registry with improved gains."""
    root = get_repository_root()
    if target_file is None:
        target_path = root / "src" / "agc" / "config" / "optimized_gains.py"
    else:
        target_path = Path(target_file)

    from ..config.optimized_gains import optimized_gains as get_current_gains

    modes = ["nominal", "euclidean", "bregman"]
    forms = ["c1", "c2"]

    registry = {}
    for m in modes:
        for f in forms:
            key = f"{m}_{f}"
            registry[key] = get_current_gains(m, f)

    # Update active key with rounded gains and persist its objective value.
    target_key = f"{mode.lower()}_{coriolis.lower()}"
    updated = round_gains(gains, sig_figs=4)
    if metadata is not None and metadata.get("cost") is not None:
        updated["optimizationCost"] = float(metadata["cost"])
    registry[target_key] = updated

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
        "def optimized_gains(mode: str, coriolis: str) -> dict:",
        '    """Return tuned per-scenario gains for each controller variant."""',
        '    key = f"{mode.lower()}_{coriolis.lower()}"',
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
        '    if key not in registry:',
        '        raise ValueError(f"Unknown mode/coriolis combination: {mode}/{coriolis}")',
        "    return registry[key]",
        "",
    ])

    target_path.write_text("\n".join(lines), encoding="utf-8")


def run_staged_optimization(
    mode: str = "bregman",
    coriolis: str = "c1",
    schedule: str = "hierarchical",
    method: str = "pso",
    polish: bool = False,
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
    variants = expand_scenario_selection(mode, coriolis)
    weights = objective_weights()
    root = get_repository_root()
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    if replay_id is not None and training_replay_ids is None:
        training_replay_ids = [replay_id]
    training_conditions = default_training_conditions(training_replay_ids, training_payload_profiles)

    overall_results = {}

    for var_idx, (sel_mode, sel_coriolis) in enumerate(variants, start=1):
        print("=" * 70)
        print(f"Optimizing {sel_mode}/{sel_coriolis} ({var_idx} of {len(variants)})")
        print(f"Schedule: {schedule} | Method: {method.upper()} | Polish: {polish} | Parallel: {parallel}")
        print(f"Training conditions: {len(training_conditions)}")
        print("=" * 70)

        # 1. Evaluate baseline seed candidates
        manual_scenarios = [
            default_scenario(
                replay_id=condition["replayId"], mode=sel_mode, coriolis=sel_coriolis,
                duration=duration, gain_source="manual", gui=False, enable_pacing=False,
                payload_profile=condition["payloadProfile"],
            )
            for condition in training_conditions
        ]
        reg_scenarios = [
            default_scenario(
                replay_id=condition["replayId"], mode=sel_mode, coriolis=sel_coriolis,
                duration=duration, gain_source="optimized", gui=False, enable_pacing=False,
                payload_profile=condition["payloadProfile"],
            )
            for condition in training_conditions
        ]

        manual_cand = encode_scenario_gains(manual_scenarios[0])
        reg_cand = encode_scenario_gains(reg_scenarios[0])

        manual_rec = evaluate_scenario_set_candidate(manual_cand, manual_scenarios, weights, label="manual")
        reg_rec = evaluate_scenario_set_candidate(reg_cand, manual_scenarios, weights, label="registered")

        incumbent = best_feasible_candidate(manual_rec, reg_rec)
        promoted = reg_rec

        # Resume from the best candidate persisted by an earlier in-place run.
        result_root = default_results_root(str(root), inplace_save=inplace_save, timestamp=stamp)
        best_gain_path = result_root / f"{sel_mode}_{sel_coriolis}.json"
        saved_best = load_best_gain(best_gain_path)
        if saved_best is not None and "candidate" in saved_best:
            saved_candidate = np.asarray(saved_best["candidate"], dtype=float)
            saved_rec = evaluate_scenario_set_candidate(
                saved_candidate, manual_scenarios, weights, label="persisted-best"
            )
            incumbent = best_feasible_candidate(incumbent, saved_rec)
            promoted = best_feasible_candidate(promoted, saved_rec)

        if not best_gain_path.exists() and not incumbent["failed"]:
            save_best_gain(
                best_gain_path, sel_mode, sel_coriolis, incumbent["gains"],
                incumbent["cost"], incumbent["label"], incumbent["candidate"],
            )

        print(f"  Manual cost:     {manual_rec['cost']:.6g} (failed={manual_rec['failed']})")
        print(f"  Registered cost: {reg_rec['cost']:.6g} (failed={reg_rec['failed']})")
        print(f"  Incumbent seed:  {incumbent['cost']:.6g} ({incumbent['label']})")

        # 2. Setup results directory
        if output_dir is not None:
            res_dir = Path(output_dir) / f"{sel_mode}_{sel_coriolis}"
        else:
            res_dir = default_results_root(
                str(root), inplace_save=inplace_save, timestamp=stamp
            ) / f"{sel_mode}_{sel_coriolis}"
        os.makedirs(res_dir, exist_ok=True)

        stages_log = []
        stage_names = gain_optimization_stages(sel_mode, schedule=schedule)
        lb_full, ub_full = gain_bounds(sel_mode)

        # 3. Staged optimization loop
        for s_idx, s_name in enumerate(stage_names, start=1):
            print(f"\n  --- Stage {s_idx}/{len(stage_names)}: {s_name} ---")

            # Collect seeds
            seeds_full = [manual_rec["candidate"], reg_rec["candidate"], incumbent["candidate"]]
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
                profile = None
                indices = gain_block_indices(sel_mode, s_name)
                lb_block = lb_full[indices]
                ub_block = ub_full[indices]

                # Project seeds to block
                block_seeds = np.unique(unique_seeds_full[:, indices], axis=0)

                # Picklable cost evaluator for block
                block_cost = BlockCostEvaluator(incumbent["candidate"], s_name, manual_scenarios, weights)

                # Run optimizer on block
                if method.lower() == "de":
                    opt = DifferentialEvolutionOptimizer(
                        cost_func=block_cost,
                        lower_bound=lb_block,
                        upper_bound=ub_block,
                        pop_size=max(5, swarm_size // len(lb_block)),
                        max_iter=max_iter,
                        tol=tol,
                        initial_points=block_seeds,
                        parallel=parallel,
                        verbose=True,
                        seed=seed,
                    )
                else:
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

                # Optional Nelder-Mead Polish on final 'all' stage
                if polish and s_name == "all":
                    best_block_x, best_cost = nelder_mead_polish(
                        cost_func=block_cost,
                        initial_point=best_block_x,
                        lower_bound=lb_block,
                        upper_bound=ub_block,
                        max_iter=40,
                        tol=1e-4,
                        verbose=True,
                    )

                # Assemble full contender
                full_cand = np.copy(incumbent["candidate"])
                full_cand[indices] = best_block_x
                contender = evaluate_scenario_set_candidate(full_cand, manual_scenarios, weights, label=s_name)
                stage_result = {
                    "method": method,
                    "block": s_name,
                    "indices": indices,
                    "history": history,
                    "best_block_x": best_block_x.tolist(),
                }

            # Update incumbent
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

            # Save stage checkpoint
            chkpt = {
                "mode": sel_mode,
                "coriolis": sel_coriolis,
                "schedule": schedule,
                "method": method,
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

            # Auto-promote strictly improved gains to registry
            if promote and is_strictly_improved(incumbent, promoted):
                print(f"    >>> PROMOTING NEW BEST GAINS for {sel_mode}/{sel_coriolis} (Cost: {incumbent['cost']:.6g} < {promoted['cost']:.6g})")
                promote_gains_to_registry(
                    mode=sel_mode,
                    coriolis=sel_coriolis,
                    gains=incumbent["gains"],
                    metadata={"cost": incumbent["cost"], "stage": s_name, "artifact": str(res_dir)},
                )
                save_best_gain(
                    best_gain_path, sel_mode, sel_coriolis, incumbent["gains"],
                    incumbent["cost"], s_name, incumbent["candidate"],
                )
                promoted = incumbent

        overall_results[f"{sel_mode}_{sel_coriolis}"] = {
            "incumbent": incumbent,
            "artifact_dir": str(res_dir),
            "stages": stages_log,
        }
        print(f"\nFinal Best Cost for {sel_mode}/{sel_coriolis}: {incumbent['cost']:.6g} (Artifacts in {res_dir})")

    return overall_results
