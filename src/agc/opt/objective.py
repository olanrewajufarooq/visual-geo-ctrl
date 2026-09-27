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
        "force": 50.0,
        "torque": 5.0,
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
        # The total effort weight remains 0.5 and is split evenly between
        # separately scaled force and torque RMS terms.
        "forceEffort": 0.25,
        "torqueEffort": 0.25,
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
    lambda_s_tracks_lambda = np.allclose(c["Lambda_s"], np.linalg.inv(c["Lambda"]))
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
    if lambda_s_tracks_lambda:
        c["Lambda_s"] = np.linalg.inv(c["Lambda"])
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
            + weights["forceEffort"] * (metrics["forceRMS"] / scales["force"]) ** 2
            + weights["torqueEffort"] * (metrics["torqueRMS"] / scales["torque"]) ** 2
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


def aggregate_scenario_records(
    records: list[Dict[str, Any]],
    label: str = "candidate",
    failure_cost: float = 1e6,
) -> Dict[str, Any]:
    """Aggregate a training set, rejecting a gain candidate when any condition fails."""
    if not records:
        raise ValueError("At least one scenario record is required for aggregation.")
    costs = np.asarray([record["cost"] for record in records], dtype=float)
    failed = any(record.get("failed", True) for record in records) or not np.all(np.isfinite(costs))
    return {
        "cost": float(failure_cost if failed else np.mean(costs)),
        "failed": bool(failed),
        "label": str(label),
        "conditionRecords": records,
    }


def evaluate_scenario_set_candidate(
    candidate: np.ndarray,
    scenarios: list[Dict[str, Any]],
    weights: Optional[Dict[str, float]] = None,
    label: str = "candidate",
) -> Dict[str, Any]:
    """Score one gain candidate across independent training conditions."""
    records = []
    for scenario in scenarios:
        record = evaluate_scenario_candidate(candidate, scenario, weights=weights, label=label)
        record["condition"] = {
            "replayId": scenario.get("replayId", "unknown"),
            "payloadProfile": scenario.get("payloadProfile", "unknown"),
        }
        records.append(record)
    failure_cost = (weights or objective_weights())["failure"]
    aggregate = aggregate_scenario_records(records, label=label, failure_cost=failure_cost)
    aggregate["candidate"] = records[0]["candidate"]
    aggregate["gains"] = records[0]["gains"]
    return aggregate


def best_feasible_candidate(incumbent: Dict[str, Any], contender: Dict[str, Any]) -> Dict[str, Any]:
    """Return the better candidate, prioritizing feasibility and strictly lower cost."""
    contender_valid = (not contender["failed"]) and np.isfinite(contender["cost"])
    incumbent_valid = (not incumbent["failed"]) and np.isfinite(incumbent["cost"])

    if contender_valid:
        if not incumbent_valid or contender["cost"] < incumbent["cost"]:
            return contender
    return incumbent
