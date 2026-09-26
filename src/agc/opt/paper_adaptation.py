"""Fair adaptation-only objective; importing this module never runs optimization."""
import numpy as np
from ..sim.publication import paper_scenario
from ..sim.paper_metrics import _pose_errors


def adaptation_objective(mode, gains):
    """Use identical frozen tracking gains/scenario/weights for both estimators.

    Caller supplies ten positive Euclidean diagonal gains or one Bregman gain.
    This opt-in entry point is deliberately not invoked by the figure pipeline.
    """
    from ..sim.run_scenario import run_scenario
    if mode not in ("euclidean", "bregman"):
        raise ValueError("Only adaptive estimators can be tuned")
    gains = np.asarray(gains, dtype=float).ravel()
    if len(gains) != (10 if mode == "euclidean" else 1) or not np.all(np.isfinite(gains)) or np.any(gains <= 0):
        return float("inf")
    scenario = paper_scenario(mode)
    scenario["controller"]["gammaE" if mode == "euclidean" else "gammaB"] = gains if mode == "euclidean" else float(gains[0])
    run, failure = run_scenario(scenario)
    if failure or run["t"][-1] < 30: return float("inf")
    p, angle = _pose_errors(run)
    if p.max() > 10 or np.max(np.abs(run["V"])) > 100: return float("inf")
    cost = np.mean(p**2 + angle**2 + .001*np.sum(run["wrench"][:, 3:]**2, axis=1)/50**2
                   + .001*np.sum(run["wrench"][:, :3]**2, axis=1)/5**2)
    return float(cost) if np.isfinite(cost) else float("inf")
