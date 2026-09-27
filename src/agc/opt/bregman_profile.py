"""1D diagnostic log-grid evaluation for scalar Bregman adaptation gain gammaB."""

from typing import Dict, Any, List, Optional, Callable
from concurrent.futures import ProcessPoolExecutor
from concurrent.futures import as_completed
import numpy as np

from .bounds import gain_bounds
from .objective import evaluate_scenario_set_candidate


def bregman_gamma_grid(seed_values: Optional[List[float]] = None) -> np.ndarray:
    """Build the log10 gammaB diagnostic profile grid spanning [-5, -1] with seeds."""
    lb, ub = gain_bounds("bregman")
    grid = np.linspace(lb[15], ub[15], 21)

    if seed_values:
        seeds = np.asarray(seed_values, dtype=float).ravel()
        valid_seeds = seeds[seeds > 0]
        if len(valid_seeds) > 0:
            log_seeds = np.clip(np.log10(valid_seeds), lb[15], ub[15])
            grid = np.concatenate([grid, log_seeds])

    return np.sort(np.unique(grid))


def _eval_bregman_worker(args):
    cand, scenarios, weights = args
    return evaluate_scenario_set_candidate(cand, scenarios, weights=weights, label="adaptive")


def profile_bregman_gain(
    base_scenario: Optional[Dict[str, Any]] = None,
    incumbent_candidate: Optional[np.ndarray] = None,
    base_scenarios: Optional[List[Dict[str, Any]]] = None,
    seed_values: Optional[List[float]] = None,
    weights: Optional[Dict[str, float]] = None,
    parallel: bool = True,
    resume_state: Optional[Dict[str, Any]] = None,
    checkpoint_callback: Optional[Callable[[Dict[str, Any]], None]] = None,
) -> Dict[str, Any]:
    """Evaluate scalar gammaB across a diagnostic grid, returning the full profile and best candidate."""
    if incumbent_candidate is None:
        raise ValueError("incumbent_candidate is required for gammaB profiling.")
    grid = bregman_gamma_grid(seed_values)

    scenarios = base_scenarios or ([base_scenario] if base_scenario is not None else [])
    if not scenarios:
        raise ValueError("At least one base scenario is required for gammaB profiling.")

    if resume_state is not None:
        saved_grid = np.asarray(resume_state.get("grid"), dtype=float)
        if not np.array_equal(np.asarray(resume_state.get("incumbent_candidate"), dtype=float), incumbent_candidate):
            raise ValueError("Bregman profile checkpoint incumbent does not match this run.")
        if saved_grid.ndim != 1 or len(saved_grid) < 1 or np.any(np.diff(saved_grid) <= 0):
            raise ValueError("Bregman profile checkpoint has an invalid grid.")
        # The first run's grid is authoritative; new external seed values must
        # not change which points an interrupted run evaluates.
        grid = saved_grid
        records = list(resume_state.get("records", []))
        if len(records) > len(grid):
            raise ValueError("Bregman profile checkpoint contains too many completed points.")
    else:
        records = []

    candidates = []
    for g in grid:
        cand = np.copy(incumbent_candidate).ravel()
        cand[15] = g
        candidates.append((cand, scenarios, weights))

    def report(index: int) -> None:
        if checkpoint_callback is not None:
            checkpoint_callback({
                "schemaVersion": 1,
                "grid": grid.copy(),
                "incumbent_candidate": np.asarray(incumbent_candidate, dtype=float).copy(),
                "records": records,
                "completedIndices": list(range(len(records))),
            })

    if parallel and len(candidates) > 1:
        with ProcessPoolExecutor() as executor:
            futures = {
                executor.submit(_eval_bregman_worker, candidates[i]): i
                for i in range(len(records), len(candidates))
            }
            # Keep an ordered prefix so the checkpoint remains simple to validate and resume.
            pending = {}
            next_index = len(records)
            for future in as_completed(futures):
                pending[futures[future]] = future.result()
                while next_index in pending:
                    records.append(pending.pop(next_index))
                    report(next_index)
                    next_index += 1
    else:
        for i in range(len(records), len(candidates)):
            records.append(_eval_bregman_worker(candidates[i]))
            report(i)

    costs = np.array([r["cost"] for r in records], dtype=float)
    failed = np.array([r["failed"] for r in records], dtype=bool)

    valid_indices = np.where((~failed) & np.isfinite(costs))[0]
    if len(valid_indices) > 0:
        best_idx = valid_indices[int(np.argmin(costs[valid_indices]))]
        best_record = records[best_idx]
    else:
        from .encoding import apply_scenario_gains, round_gains
        incumbent_gains = apply_scenario_gains(incumbent_candidate, scenarios[0])["controller"]
        best_record = {
            "candidate": incumbent_candidate,
            "gains": round_gains(incumbent_gains),
            "cost": float("inf"),
            "failed": True,
            "label": "adaptive",
        }

    return {
        "logGammaB": grid.tolist(),
        "gammaB": (10.0 ** grid).tolist(),
        "cost": costs.tolist(),
        "failed": failed.tolist(),
        "best": best_record,
    }
