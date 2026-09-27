"""Metrics used by the paper experiment and figure workflow."""

from typing import Any, Dict, Optional
import numpy as np


def _pose_errors(run: Dict[str, Any]) -> tuple[np.ndarray, np.ndarray]:
    H = np.asarray(run["H"], dtype=float)
    Hd = np.asarray(run["Hdesired"], dtype=float)
    position = np.linalg.norm(H[:, :3, 3] - Hd[:, :3, 3], axis=1)
    relative = np.einsum("nij,njk->nik", np.transpose(Hd[:, :3, :3], (0, 2, 1)), H[:, :3, :3])
    cosine = np.clip((np.trace(relative, axis1=1, axis2=2) - 1.0) / 2.0, -1.0, 1.0)
    attitude = np.arccos(cosine)
    return position, attitude


def _window_mask(t: np.ndarray, start_time: float, end_time: float) -> np.ndarray:
    if end_time < start_time:
        raise ValueError("end_time must be greater than or equal to start_time")
    mask = (t >= float(start_time)) & (t <= float(end_time))
    if not np.any(mask):
        raise ValueError("requested metric window contains no samples")
    return mask


def compute_window_metrics(run: Dict[str, Any], start_time: float, end_time: float) -> Dict[str, float]:
    """Compute unit-aware tracking and commanded-wrench metrics over a time window."""
    t = np.asarray(run["t"], dtype=float)
    mask = _window_mask(t, start_time, end_time)
    position, attitude = _pose_errors(run)
    wrench = np.asarray(run["wrench"], dtype=float)
    force_norm = np.linalg.norm(wrench[:, 3:6], axis=1)
    torque_norm = np.linalg.norm(wrench[:, :3], axis=1)
    sliding = np.asarray(run.get("s", np.zeros((len(t), 6))), dtype=float)
    sliding_norm = np.linalg.norm(sliding, axis=1)
    selected_t = t[mask]
    selected_force = wrench[mask, 3:]
    selected_torque = wrench[mask, :3]
    integrated_force = float(np.trapezoid(np.sum(selected_force ** 2, axis=1), selected_t)) if len(selected_t) > 1 else 0.0
    integrated_torque = float(np.trapezoid(np.sum(selected_torque ** 2, axis=1), selected_t)) if len(selected_t) > 1 else 0.0
    return {
        "startTime": float(start_time),
        "endTime": float(end_time),
        "positionRMSE": float(np.sqrt(np.mean(position[mask] ** 2))),
        "attitudeRMSE": float(np.sqrt(np.mean(attitude[mask] ** 2))),
        "maxPositionError": float(np.max(position[mask])),
        "maxAttitudeError": float(np.max(attitude[mask])),
        "maxSlidingNorm": float(np.max(sliding_norm[mask])),
        "forceRMS": float(np.sqrt(np.mean(force_norm[mask] ** 2))),
        "torqueRMS": float(np.sqrt(np.mean(torque_norm[mask] ** 2))),
        "peakForce": float(np.max(force_norm[mask])),
        "peakTorque": float(np.max(torque_norm[mask])),
        "integratedSquaredForce": integrated_force,
        "integratedSquaredTorque": integrated_torque,
        "maxPsi": float(np.max(np.asarray(run["Psi"])[mask])),
        "maxVs": float(np.max(np.asarray(run["Vs"])[mask])),
    }


def compute_recovery_time(
    run: Dict[str, Any],
    release_time: float,
    position_tolerance: float = 0.05,
    attitude_tolerance: float = np.deg2rad(5.0),
    dwell_time: float = 1.0,
) -> Optional[float]:
    """Return first post-release time satisfying tracking thresholds for a dwell interval."""
    if position_tolerance <= 0.0 or attitude_tolerance <= 0.0 or dwell_time <= 0.0:
        raise ValueError("recovery tolerances and dwell_time must be positive")
    t = np.asarray(run["t"], dtype=float)
    position, attitude = _pose_errors(run)
    eligible = np.flatnonzero(t >= float(release_time))
    for index in eligible:
        end = np.searchsorted(t, t[index] + dwell_time, side="left")
        if end == len(t):
            continue
        if end > index and np.all(position[index:end+1] <= position_tolerance) and np.all(attitude[index:end+1] <= attitude_tolerance):
            return float(t[index])
    return None


def compute_reaching_time(
    run: Dict[str, Any],
    threshold: float = 1e-3,
    dwell_time: float = 0.5,
    lambda_s: Optional[np.ndarray] = None,
) -> Optional[float]:
    """Return the first time the composite-error norm stays below threshold."""
    if threshold <= 0.0 or dwell_time <= 0.0:
        raise ValueError("threshold and dwell_time must be positive")
    t = np.asarray(run["t"], dtype=float)
    vectors = np.asarray(run["s"], dtype=float)
    metric = np.eye(6) if lambda_s is None else lambda_s
    s = np.sqrt(np.einsum("ni,ij,nj->n", vectors, metric, vectors))
    for index in np.flatnonzero(s <= threshold):
        end = np.searchsorted(t, t[index] + dwell_time, side="left")
        if end == len(t):
            continue
        if end > index and np.all(s[index:end+1] <= threshold):
            return float(t[index])
    return None


def compute_nominal_reaching_bound(
    run: Dict[str, Any],
    inertia: np.ndarray,
    lambda_s: np.ndarray,
    ks: float,
    alpha: float,
) -> float:
    """Compute the finite-time certificate from the initial transverse energy."""
    if ks <= 0.0 or not 0.0 < alpha < 1.0:
        raise ValueError("ks must be positive and alpha must lie in (0, 1)")
    inertia = np.asarray(inertia, dtype=float)
    lambda_s = np.asarray(lambda_s, dtype=float)
    initial_s = np.asarray(run["s"], dtype=float)[0]
    initial_vs = float(0.5 * initial_s @ inertia @ initial_s)
    q = (1.0 + alpha) / 2.0
    c_lambda = 2.0 * np.min(np.linalg.eigvalsh(lambda_s)) / np.max(np.linalg.eigvalsh(inertia))
    c_alpha = ks * c_lambda ** q
    return float(initial_vs ** (1.0 - q) / (c_alpha * (1.0 - q))) if initial_vs > 0.0 else 0.0
