"""Scenario and trajectory validation utilities ensuring simulation integrity."""

from typing import Dict, Any, Callable
import numpy as np

from ..math.inertia import is_spd, inertia_from_pi


def validate_scenario(scenario: Dict[str, Any]) -> None:
    """Validate closed-loop simulation scenario configuration.

    Raises:
    -------
    ValueError : if any structural, timing, physical, or numerical requirement is violated.
    KeyError   : if any required field is missing.
    """
    required_keys = [
        "duration",
        "dtPlant",
        "dtControl",
        "dtAdaptation",
        "plantPi",
        "plantGravity",
        "initial",
        "initialEstimate",
        "controller",
        "trajectory",
    ]
    for k in required_keys:
        if k not in scenario:
            raise KeyError(f"Scenario missing required key: {k!r}")

    # 1. Timing validation
    duration = float(scenario["duration"])
    dt_plant = float(scenario["dtPlant"])
    dt_control = float(scenario["dtControl"])
    dt_adapt = float(scenario["dtAdaptation"])

    for name, val in [
        ("duration", duration),
        ("dtPlant", dt_plant),
        ("dtControl", dt_control),
        ("dtAdaptation", dt_adapt),
    ]:
        if not np.isfinite(val) or val <= 0.0:
            raise ValueError(f"Scenario timing parameter {name!r} must be positive and finite, got {val}")

    # Integer timestep ratios
    ctrl_ratio = dt_control / dt_plant
    if abs(round(ctrl_ratio) - ctrl_ratio) > 1e-6 or round(ctrl_ratio) < 1:
        raise ValueError(
            f"dtControl ({dt_control}) must be an integer multiple of dtPlant ({dt_plant}), got ratio {ctrl_ratio}"
        )

    adapt_ratio = dt_adapt / dt_plant
    if abs(round(adapt_ratio) - adapt_ratio) > 1e-6 or round(adapt_ratio) < 1:
        raise ValueError(
            f"dtAdaptation ({dt_adapt}) must be an integer multiple of dtPlant ({dt_plant}), got ratio {adapt_ratio}"
        )

    dur_ratio = duration / dt_plant
    if abs(round(dur_ratio) - dur_ratio) > 1e-6 or round(dur_ratio) < 1:
        raise ValueError(
            f"duration ({duration}) must be an integer multiple of dtPlant ({dt_plant}), got ratio {dur_ratio}"
        )

    # 2. Plant parameters
    plant_pi = np.asarray(scenario["plantPi"], dtype=float).ravel()
    if len(plant_pi) != 10 or not np.all(np.isfinite(plant_pi)):
        raise ValueError(f"plantPi must be a finite 10-element vector, got shape {plant_pi.shape}")
    if plant_pi[0] <= 0.0:
        raise ValueError(f"plantPi mass must be strictly positive, got {plant_pi[0]}")
    J_plant = inertia_from_pi(plant_pi)
    if not is_spd(J_plant):
        raise ValueError("plantPi inertia submatrix must be symmetric positive definite.")

    gravity = np.asarray(scenario["plantGravity"], dtype=float).ravel()
    if len(gravity) != 3 or not np.all(np.isfinite(gravity)):
        raise ValueError(f"plantGravity must be a finite 3-element vector, got {gravity}")

    # 3. Payload drop validation
    payload_drop = scenario.get("payloadDrop")
    if payload_drop is not None:
        if not isinstance(payload_drop, dict):
            raise ValueError("payloadDrop must be a dictionary when specified.")
        for pk in ["payload", "releaseTime", "loadedPi", "barePi"]:
            if pk not in payload_drop:
                raise KeyError(f"payloadDrop missing required key: {pk!r}")

        rel_time = float(payload_drop["releaseTime"])
        if not np.isfinite(rel_time) or rel_time <= 0.0:
            raise ValueError(f"payloadDrop releaseTime must be strictly positive, got {rel_time}")

        rel_ratio = rel_time / dt_plant
        if abs(round(rel_ratio) - rel_ratio) > 1e-6:
            raise ValueError(
                f"payloadDrop releaseTime ({rel_time}) must be aligned to dtPlant ({dt_plant})"
            )

        bare_pi = np.asarray(payload_drop["barePi"], dtype=float).ravel()
        loaded_pi = np.asarray(payload_drop["loadedPi"], dtype=float).ravel()
        for pi_name, pi_vec in [("barePi", bare_pi), ("loadedPi", loaded_pi)]:
            if len(pi_vec) != 10 or not np.all(np.isfinite(pi_vec)):
                raise ValueError(f"{pi_name} must be a finite 10-element vector.")
            if pi_vec[0] <= 0.0:
                raise ValueError(f"{pi_name} mass must be positive.")
            if not is_spd(inertia_from_pi(pi_vec)):
                raise ValueError(f"{pi_name} inertia matrix must be symmetric positive definite.")

        p_info = payload_drop["payload"]
        if "mass" not in p_info or float(p_info["mass"]) <= 0.0:
            raise ValueError("payload mass must be positive.")
        if abs(loaded_pi[0] - (bare_pi[0] + float(p_info["mass"]))) > 1e-4:
            raise ValueError("loadedPi mass must equal barePi mass plus payload mass.")

    # 4. Controller validation
    c = scenario["controller"]
    if not isinstance(c, dict):
        raise ValueError("controller must be a dictionary.")
    mode = str(c.get("mode", "")).lower()
    if mode not in {"nominal", "euclidean", "bregman"}:
        raise ValueError(f"Invalid controller mode: {mode!r}. Must be 'nominal', 'euclidean', or 'bregman'.")

    coriolis = str(c.get("coriolis", "")).lower()
    if coriolis not in {"c1", "c2"}:
        raise ValueError(f"Invalid Coriolis factorization: {coriolis!r}. Must be 'c1' or 'c2'.")

    for gain_name in ["KR", "Kxi"]:
        if gain_name not in c:
            raise KeyError(f"controller missing gain matrix: {gain_name!r}")
        mat = np.asarray(c[gain_name], dtype=float)
        if mat.shape != (3, 3) or not is_spd(mat):
            raise ValueError(f"controller gain {gain_name} must be a 3x3 symmetric positive definite matrix.")

    if "Lambda" not in c:
        raise KeyError("controller missing gain matrix: 'Lambda'")
    mat_lambda = np.asarray(c["Lambda"], dtype=float)
    if mat_lambda.shape != (6, 6) or not is_spd(mat_lambda):
        raise ValueError("controller gain Lambda must be a 6x6 symmetric positive definite matrix.")

    for scalar_name in ["kd", "ks", "alpha"]:
        if scalar_name not in c:
            raise KeyError(f"controller missing scalar gain: {scalar_name!r}")
        val = float(c[scalar_name])
        if not np.isfinite(val) or val <= 0.0:
            raise ValueError(f"controller scalar gain {scalar_name} must be positive, got {val}")

    if float(c["alpha"]) > 1.0:
        raise ValueError(f"controller dissipation exponent alpha must satisfy 0 < alpha <= 1, got {c['alpha']}")

    # Initial estimate validation
    est = scenario["initialEstimate"]
    if mode == "bregman":
        J_est = np.asarray(est, dtype=float)
        if J_est.shape != (4, 4) or not is_spd(J_est):
            raise ValueError("bregman initialEstimate must be a 4x4 symmetric positive definite matrix.")
    else:
        pi_est = np.asarray(est, dtype=float).ravel()
        if len(pi_est) != 10 or not np.all(np.isfinite(pi_est)):
            raise ValueError(f"{mode} initialEstimate must be a finite 10-element vector.")
        if pi_est[0] <= 0.0:
            raise ValueError("initialEstimate mass must be positive.")

    # 5. Initial state validation
    init = scenario["initial"]
    if not isinstance(init, dict) or "H" not in init or "V" not in init:
        raise KeyError("initial must be a dictionary with 'H' and 'V'.")
    H_init = np.asarray(init["H"], dtype=float)
    if H_init.shape != (4, 4) or not np.all(np.isfinite(H_init)):
        raise ValueError("initial H must be a finite 4x4 matrix.")
    R_init = H_init[0:3, 0:3]
    if np.linalg.norm(R_init.T @ R_init - np.eye(3)) > 1e-4 or np.linalg.det(R_init) <= 0.0:
        raise ValueError("initial H rotation block must belong to SO(3).")
    if np.linalg.norm(H_init[3, :] - np.array([0.0, 0.0, 0.0, 1.0])) > 1e-6:
        raise ValueError("initial H last row must be [0, 0, 0, 1].")

    V_init = np.asarray(init["V"], dtype=float).ravel()
    if len(V_init) != 6 or not np.all(np.isfinite(V_init)):
        raise ValueError("initial V must be a finite 6-element vector.")

    # 6. Trajectory validation
    traj = scenario["trajectory"]
    if not callable(traj):
        raise ValueError("trajectory must be a callable returning desired state at time t.")
    try:
        sample0 = traj(0.0)
    except Exception as exc:
        raise ValueError(f"trajectory failed to evaluate at t = 0.0: {exc}") from exc
    if not isinstance(sample0, dict) or "H" not in sample0 or "V" not in sample0:
        raise ValueError("trajectory sample must return a dictionary with 'H' and 'V'.")


def validate_replay_data(traj_data: Dict[str, Any]) -> None:
    """Validate raw replay trajectory dictionary parsed from .mat file."""
    required = ["t", "p", "v_b", "a_b", "omega_b", "alpha_b", "R"]
    for k in required:
        if k not in traj_data:
            raise KeyError(f"Replay trajectory data missing required field: {k!r}")

    t = np.asarray(traj_data["t"], dtype=float).ravel()
    n = len(t)
    if n < 2:
        raise ValueError(f"Replay trajectory must contain at least 2 samples, got {n}")
    if not np.all(np.isfinite(t)):
        raise ValueError("Replay trajectory time vector must be finite.")
    if np.any(np.diff(t) <= 0.0):
        raise ValueError("Replay trajectory time vector must be strictly monotonically increasing.")

    for field in ["p", "v_b", "a_b", "omega_b", "alpha_b"]:
        arr = np.asarray(traj_data[field], dtype=float)
        if arr.shape != (n, 3):
            raise ValueError(f"Replay field {field} must have shape ({n}, 3), got {arr.shape}")
        if not np.all(np.isfinite(arr)):
            raise ValueError(f"Replay field {field} contains NaN or Inf.")

    R = np.asarray(traj_data["R"], dtype=float)
    if R.shape == (3, 3, n):
        # Transpose to (n, 3, 3) for uniform indexing if needed
        for i in range(min(5, n)):
            Ri = R[:, :, i]
            if np.linalg.norm(Ri.T @ Ri - np.eye(3)) > 1e-3:
                raise ValueError(f"Replay orientation at sample {i} is not a valid rotation matrix.")
    elif R.shape == (n, 3, 3):
        for i in range(min(5, n)):
            Ri = R[i, :, :]
            if np.linalg.norm(Ri.T @ Ri - np.eye(3)) > 1e-3:
                raise ValueError(f"Replay orientation at sample {i} is not a valid rotation matrix.")
    else:
        raise ValueError(f"Replay field 'R' must have shape (3, 3, {n}) or ({n}, 3, 3), got {R.shape}")
