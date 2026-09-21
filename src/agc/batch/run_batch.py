"""Batch simulation execution for paper comparison variants."""

from typing import List, Dict, Any
from ..sim.run_scenario import run_scenario
from ..sim.metrics import compute_metrics


def run_batch(scenarios: List[dict]) -> Dict[str, Any]:
    """Execute multiple scenarios and aggregate runs and metrics."""
    runs = []
    metrics_list = []
    failures = []

    for i, scen in enumerate(scenarios):
        mode = scen["controller"]["mode"]
        coriolis = scen["controller"]["coriolis"]
        print(f"[{i+1}/{len(scenarios)}] Simulating {mode}_{coriolis}...")

        run, failure = run_scenario(scen)
        runs.append(run)
        failures.append(failure)

        if failure is None:
            met = compute_metrics(run)
            metrics_list.append(met)
            print(f"    --> Succeeded: pos RMSE = {met['positionRMSE']:.4f} m, att RMSE = {met['attitudeRMSE']:.4f} rad")
        else:
            metrics_list.append(None)
            print(f"    --> Failed at t = {failure['time']:.3f} s: {failure['message']}")

    return {
        "runs": runs,
        "metrics": metrics_list,
        "failures": failures,
        "successCount": sum(1 for f in failures if f is None),
    }
