"""Validation utilities for nominal scenarios and replay trajectories."""

from typing import Any, Dict
import numpy as np

from ..math.inertia import inertia_from_pi, is_spd


def validate_scenario(scenario: Dict[str, Any]) -> None:
    required = ["duration", "dtPlant", "dtControl", "plantPi", "plantGravity", "initial", "controller", "trajectory"]
    for key in required:
        if key not in scenario:
            raise KeyError(f"Scenario missing required key: {key!r}")
    duration, dt_plant, dt_control = (float(scenario[k]) for k in ("duration", "dtPlant", "dtControl"))
    for name, value in (("duration", duration), ("dtPlant", dt_plant), ("dtControl", dt_control)):
        if not np.isfinite(value) or value <= 0:
            raise ValueError(f"Scenario timing parameter {name!r} must be positive and finite")
    for name, ratio in (("dtControl", dt_control / dt_plant), ("duration", duration / dt_plant)):
        if abs(round(ratio) - ratio) > 1e-6 or round(ratio) < 1:
            raise ValueError(f"{name} must be an integer multiple of dtPlant")
    pi = np.asarray(scenario["plantPi"], dtype=float).ravel()
    if pi.shape != (10,) or not np.all(np.isfinite(pi)) or pi[0] <= 0 or not is_spd(inertia_from_pi(pi)):
        raise ValueError("plantPi must be a finite physical 10-parameter inertia vector")
    gravity = np.asarray(scenario["plantGravity"], dtype=float).ravel()
    if gravity.shape != (3,) or not np.all(np.isfinite(gravity)):
        raise ValueError("plantGravity must be a finite 3-element vector")
    cfg = scenario["controller"]
    if cfg.get("mode") != "nominal" or cfg.get("coriolis") not in {"lc", "rb"}:
        raise ValueError("Only nominal LC and RB controllers are supported")
    for name, shape in (("KR", (3, 3)), ("Kxi", (3, 3)), ("Lambda", (6, 6))):
        matrix = np.asarray(cfg.get(name), dtype=float)
        if matrix.shape != shape or not is_spd(matrix):
            raise ValueError(f"controller gain {name} must be a positive-definite {shape} matrix")
    for name in ("kd", "ks", "alpha"):
        if not np.isfinite(float(cfg.get(name, np.nan))) or float(cfg[name]) <= 0:
            raise ValueError(f"controller scalar gain {name} must be positive")
    if not 0 < float(cfg["alpha"]) < 1:
        raise ValueError("controller dissipation exponent alpha must satisfy 0 < alpha < 1")
    init = scenario["initial"]
    H = np.asarray(init.get("H"), dtype=float)
    V = np.asarray(init.get("V"), dtype=float).ravel()
    if H.shape != (4, 4) or V.shape != (6,) or not np.all(np.isfinite(H)) or not np.all(np.isfinite(V)):
        raise ValueError("initial state must contain finite H(4,4) and V(6)")
    if not callable(scenario["trajectory"]):
        raise ValueError("trajectory must be callable")


def validate_replay_data(traj_data: Dict[str, Any]) -> None:
    required = ["t", "p", "v_b", "a_b", "omega_b", "alpha_b", "R"]
    for key in required:
        if key not in traj_data:
            raise KeyError(f"Replay trajectory data missing required field: {key!r}")
    t = np.asarray(traj_data["t"], dtype=float).ravel()
    n = len(t)
    if n < 2 or not np.all(np.isfinite(t)) or np.any(np.diff(t) <= 0):
        raise ValueError("Replay trajectory time vector must be finite and strictly increasing")
    for field in ("p", "v_b", "a_b", "omega_b", "alpha_b"):
        arr = np.asarray(traj_data[field], dtype=float)
        if arr.shape != (n, 3) or not np.all(np.isfinite(arr)):
            raise ValueError(f"Replay field {field} must have finite shape ({n}, 3)")
    R = np.asarray(traj_data["R"], dtype=float)
    if R.shape == (3, 3, n):
        samples = (R[:, :, i] for i in range(min(5, n)))
    elif R.shape == (n, 3, 3):
        samples = (R[i] for i in range(min(5, n)))
    else:
        raise ValueError(f"Replay field R has unexpected shape {R.shape}")
    for i, matrix in enumerate(samples):
        if np.linalg.norm(matrix.T @ matrix - np.eye(3)) > 1e-3:
            raise ValueError(f"Replay orientation at sample {i} is not a valid rotation matrix")
