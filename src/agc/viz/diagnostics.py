"""MATLAB-compatible derived signals for saved PyBullet runs."""

from typing import Any, Dict, Tuple
import numpy as np

from ..math.inertia import pseudo_from_pi


def _rpy_zyx(rotations: np.ndarray) -> np.ndarray:
    """Return continuous roll/pitch/yaw angles for rotation matrices."""
    r = np.asarray(rotations, dtype=float)
    pitch = np.arcsin(np.clip(-r[:, 2, 0], -1.0, 1.0))
    roll = np.arctan2(r[:, 2, 1], r[:, 2, 2])
    yaw = np.arctan2(r[:, 1, 0], r[:, 0, 0])
    return np.unwrap(np.column_stack((roll, pitch, yaw)), axis=0)


def _active_parameters(run: Dict[str, Any], scenario: Dict[str, Any], n: int) -> np.ndarray:
    if "activePlantPi" in run and len(run["activePlantPi"]) == n:
        return np.asarray(run["activePlantPi"], dtype=float)
    return np.repeat(np.asarray(scenario["plantPi"], dtype=float).reshape(1, -1), n, axis=0)


def derive_diagnostics(run: Dict[str, Any], scenario: Dict[str, Any]) -> Dict[str, np.ndarray]:
    """Derive all signals consumed by standalone and comparison figures."""
    t = np.asarray(run["t"], dtype=float)
    h = np.asarray(run["H"], dtype=float)
    hd = np.asarray(run["Hdesired"], dtype=float)
    n = len(t)
    position = h[:, :3, 3]
    desired_position = hd[:, :3, 3]
    rpy = _rpy_zyx(h[:, :3, :3])
    desired_rpy = _rpy_zyx(hd[:, :3, :3])
    re = np.einsum("nij,njk->nik", np.transpose(hd[:, :3, :3], (0, 2, 1)), h[:, :3, :3])
    attitude_error = np.arccos(np.clip((np.trace(re, axis1=1, axis2=2) - 1.0) / 2.0, -1.0, 1.0))
    controller = scenario.get("controller", {})
    lam = np.asarray(controller.get("Lambda", np.eye(6)), dtype=float)
    lam_inv = np.linalg.inv(lam)
    s = np.asarray(run["s"], dtype=float)
    sliding_norm = np.sqrt(np.maximum(0.0, np.einsum("ni,ij,nj->n", s, lam_inv, s)))
    estimate = np.asarray(run["estimatePi"], dtype=float)
    truth = _active_parameters(run, scenario, n)
    mass = np.maximum(estimate[:, 0], 1e-12)
    true_mass = np.maximum(truth[:, 0], 1e-12)
    cog = estimate[:, 1:4] / mass[:, None]
    true_cog = truth[:, 1:4] / true_mass[:, None]
    pseudo_margin = np.array([np.min(np.linalg.eigvalsh(pseudo_from_pi(p))) for p in estimate])
    parameter_error = np.linalg.norm(estimate - truth, axis=1) / max(float(np.linalg.norm(truth[0])), 1.0)
    return {
        "position": position,
        "desiredPosition": desired_position,
        "rpy": rpy,
        "desiredRpy": desired_rpy,
        "positionError": np.linalg.norm(position - desired_position, axis=1),
        "attitudeError": attitude_error,
        "slidingNorm": sliding_norm,
        "Vs": np.asarray(run["Vs"], dtype=float),
        "wrenchNorm": np.linalg.norm(np.asarray(run["wrench"], dtype=float), axis=1),
        "estimatePi": estimate,
        "truePi": truth,
        "parameterError": parameter_error,
        "pseudoMargin": pseudo_margin,
        "cog": cog,
        "trueCog": true_cog,
    }


def slice_release_window(
    run: Dict[str, Any], diagnostics: Dict[str, np.ndarray], release_time: float
) -> Tuple[Dict[str, Any], Dict[str, np.ndarray]]:
    """Return aligned arrays starting at payload release with time reset to zero."""
    t = np.asarray(run["t"], dtype=float)
    index = int(np.searchsorted(t, release_time, side="left"))
    run_slice = dict(run)
    for key, value in run.items():
        if isinstance(value, np.ndarray) and value.ndim > 0 and len(value) == len(t):
            run_slice[key] = value[index:]
    run_slice["t"] = t[index:] - release_time
    diag_slice = {key: value[index:] for key, value in diagnostics.items() if isinstance(value, np.ndarray) and len(value) == len(t)}
    return run_slice, diag_slice
