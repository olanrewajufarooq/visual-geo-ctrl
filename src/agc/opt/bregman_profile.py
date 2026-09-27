"""1D diagnostic log-grid evaluation for scalar Bregman adaptation gain gammaB."""

from typing import Dict, Any, List, Optional
from concurrent.futures import ProcessPoolExecutor
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
) -> Dict[str, Any]:
    """Evaluate scalar gammaB across a diagnostic grid, returning the full profile and best candidate."""
    if incumbent_candidate is None:
        raise ValueError("incumbent_candidate is required for gammaB profiling.")
    grid = bregman_gamma_grid(seed_values)

    scenarios = base_scenarios or ([base_scenario] if base_scenario is not None else [])
    if not scenarios:
        raise ValueError("At least one base scenario is required for gammaB profiling.")

    candidates = []
    for g in grid:
        cand = np.copy(incumbent_candidate).ravel()
        cand[15] = g
        candidates.append((cand, scenarios, weights))

    if parallel and len(candidates) > 1:
        with ProcessPoolExecutor() as executor:
            records = list(executor.map(_eval_bregman_worker, candidates))
    else:
        records = [_eval_bregman_worker(c) for c in candidates]

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
