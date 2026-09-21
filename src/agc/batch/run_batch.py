"""Batch simulation execution for paper comparison variants with serial and parallel support."""

from typing import List, Dict, Any, Optional, Tuple
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from ..sim.run_scenario import run_scenario
from ..sim.metrics import compute_metrics


def _run_single_scenario(scenario: dict) -> Tuple[dict, Optional[dict]]:
    """Worker function executing one scenario with full failure isolation."""
    try:
        return run_scenario(scenario)
    except Exception as exc:
        empty_run = {
            "t": np.array([0.0]),
            "mode": str(scenario.get("controller", {}).get("mode", "unknown")),
            "coriolis": str(scenario.get("controller", {}).get("coriolis", "unknown")),
        }
        failure = {
            "identifier": type(exc).__name__,
            "message": str(exc),
            "time": 0.0,
        }
        return empty_run, failure


def run_batch(
    scenarios: List[dict],
    parallel: bool = False,
    max_workers: Optional[int] = None,
    verbose: bool = True,
) -> Dict[str, Any]:
    """Execute multiple scenarios and aggregate runs and metrics.

    Parameters:
    -----------
    scenarios : list of dict
        Simulation scenarios to execute.
    parallel : bool
        If True, executes scenarios in parallel across worker processes.
    max_workers : int, optional
        Maximum worker processes for parallel execution.
    verbose : bool
        If True, prints progress updates.

    Returns:
    --------
    dict with 'runs', 'metrics', 'failures', 'successCount', and 'parallel'.
    """
    n = len(scenarios)
    runs: List[dict] = [None] * n  # type: ignore
    failures: List[Optional[dict]] = [None] * n
    metrics_list: List[Optional[dict]] = [None] * n

    if parallel and n > 1:
        if verbose:
            print(f"Executing batch of {n} scenarios in parallel...")
        with ProcessPoolExecutor(max_workers=max_workers) as executor:
            results = list(executor.map(_run_single_scenario, scenarios))
        for i, (run, failure) in enumerate(results):
            runs[i] = run
            failures[i] = failure
    else:
        for i, scen in enumerate(scenarios):
            mode = scen["controller"]["mode"]
            coriolis = scen["controller"]["coriolis"]
            if verbose:
                print(f"[{i+1}/{n}] Simulating {mode}_{coriolis}...")
            run, failure = _run_single_scenario(scen)
            runs[i] = run
            failures[i] = failure

    # Compute metrics for completed runs
    for i in range(n):
        if failures[i] is None:
            met = compute_metrics(runs[i])
            metrics_list[i] = met
            if verbose:
                mode = scenarios[i]["controller"]["mode"]
                coriolis = scenarios[i]["controller"]["coriolis"]
                print(f"    --> {mode}_{coriolis}: pos RMSE = {met['positionRMSE']:.4f} m, att RMSE = {met['attitudeRMSE']:.4f} rad")
        else:
            metrics_list[i] = None
            if verbose:
                mode = scenarios[i]["controller"]["mode"]
                coriolis = scenarios[i]["controller"]["coriolis"]
                fail = failures[i]
                print(f"    --> {mode}_{coriolis} FAILED at t = {fail['time']:.3f} s: {fail['message']}")

    success_count = sum(1 for f in failures if f is None)
    return {
        "runs": runs,
        "metrics": metrics_list,
        "failures": failures,
        "successCount": success_count,
        "parallel": bool(parallel),
    }
