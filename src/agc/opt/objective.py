"""Objective function, dimensionless error scales, and candidate scoring."""

from typing import Dict, Any, Optional
import warnings
import numpy as np

from .encoding import apply_scenario_gains, encode_scenario_gains, round_gains
from ..sim.run_scenario import run_scenario
from ..sim.metrics import compute_metrics


def objective_scales() -> Dict[str, float]:
    """Return acceptable physical error scales for dimensionless costs."""
    return {
        "position": 0.05,
        "attitude": 0.05,
        "mass": 0.05,
        "cog": 0.01,
        "linVel": 0.20,
        "angVel": 0.50,
        "inertia": 0.05,
        "effort": 50.0,
    }


def objective_weights() -> Dict[str, float]:
    """Return shared optimizer trade-off weights."""
    return {
        "position": 2.0,
        "attitude": 2.0,
        "mass": 2.5,
        "cog": 2.5,
        "linVel": 0.5,
        "angVel": 0.5,
        "inertia": 1.5,
        "effort": 0.01,
        "failure": 1e6,
    }


def optimization_options(options: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """Merge user options with standard optimizer defaults."""
    opts = {
        "weights": objective_weights(),
        "scales": objective_scales(),
        "swarmSize": 50,
        "maxIterations": 20,
        "functionTolerance": 1e-3,
        "maxStallIterations": 10,
        "parallel": True,
        "initialPoints": None,
    }
    if options:
        opts.update(options)
        if "weights" in options and isinstance(options["weights"], dict):
            w = objective_weights()
            w.update(options["weights"])
            opts["weights"] = w
    return opts


def evaluate_scenario_candidate(
    candidate: np.ndarray,
    base_scenario: Dict[str, Any],
    weights: Optional[Dict[str, float]] = None,
    label: str = "candidate",
) -> Dict[str, Any]:
    """Round and score one complete gain candidate in closed-loop PyBullet simulation."""
    if weights is None:
        weights = objective_weights()
    scales = objective_scales()

    # 1. Decode and round candidate to 4 significant figures
    temp_scenario = apply_scenario_gains(candidate, base_scenario)
    c = temp_scenario["controller"]
    raw_gains = {
        "KRdiag": np.diag(c["KR"]),
        "Kxidiag": np.diag(c["Kxi"]),
        "LambdaDiag": np.diag(c["Lambda"]),
        "kd": float(c["kd"]),
        "ks": float(c["ks"]),
        "alpha": float(c["alpha"]),
        "gammaE": c.get("gammaE", None),
        "gammaB": c.get("gammaB", None),
    }
    gains = round_gains(raw_gains, sig_figs=4)

    # 2. Build canonical scenario with rounded gains
    c["KR"] = np.diag(gains["KRdiag"])
    c["Kxi"] = np.diag(gains["Kxidiag"])
    c["Lambda"] = np.diag(gains["LambdaDiag"])
    c["kd"] = gains["kd"]
    c["ks"] = gains["ks"]
    c["alpha"] = gains["alpha"]
    if gains["gammaE"] is not None:
        c["gammaE"] = gains["gammaE"]
    if gains["gammaB"] is not None:
        c["gammaB"] = gains["gammaB"]
    temp_scenario["controller"] = c
    temp_scenario["gui"] = False  # Headless evaluation
    temp_scenario["enable_pacing"] = False  # Never wall-clock pace optimizer trials

    published_candidate = encode_scenario_gains(temp_scenario)

    # 3. Execute closed-loop simulation
    with warnings.catch_warnings(), np.errstate(all="ignore"):
        warnings.simplefilter("ignore", category=RuntimeWarning)
        run, failure = run_scenario(temp_scenario)

    if failure is None:
        metrics = compute_metrics(run)
        cost = (
            weights["position"] * (metrics["positionRMSE"] / scales["position"]) ** 2
            + weights["attitude"] * (metrics["attitudeRMSE"] / scales["attitude"]) ** 2
            + weights["mass"] * (metrics["massEstimationRMSE"] / scales["mass"]) ** 2
            + weights["cog"] * (metrics["centerOfMassEstimationRMSE"] / scales["cog"]) ** 2
            + weights["linVel"] * (metrics["linearVelocityRMSE"] / scales["linVel"]) ** 2
            + weights["angVel"] * (metrics["angularVelocityRMSE"] / scales["angVel"]) ** 2
            + weights["inertia"] * (metrics["inertiaEstimationRMSE"] / scales["inertia"]) ** 2
            + weights["effort"] * (metrics["wrenchRMS"] / scales["effort"]) ** 2
        )
        failed = not np.isfinite(cost)
    else:
        cost = float(weights["failure"])
        failed = True
        metrics = {}

    return {
        "rawCandidate": np.asarray(candidate, dtype=float).ravel(),
        "candidate": published_candidate,
        "gains": gains,
        "cost": float(cost),
        "failed": failed,
        "label": str(label),
        "metrics": metrics,
        "failure": failure,
    }


def best_feasible_candidate(incumbent: Dict[str, Any], contender: Dict[str, Any]) -> Dict[str, Any]:
    """Return the better candidate, prioritizing feasibility and strictly lower cost."""
    contender_valid = (not contender["failed"]) and np.isfinite(contender["cost"])
    incumbent_valid = (not incumbent["failed"]) and np.isfinite(incumbent["cost"])

    if contender_valid:
        if not incumbent_valid or contender["cost"] < incumbent["cost"]:
            return contender
    return incumbent
