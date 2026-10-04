"""Performance and tracking metrics calculation."""

from typing import Dict, Any
import numpy as np
from ..math.se3 import adjoint_se3, inv_se3


def compute_metrics(run: Dict[str, Any]) -> Dict[str, float]:
    """Compute stable, unit-aware tracking and theory diagnostics."""
    n = len(run["t"])
    pos_error = np.zeros((n, 3), dtype=float)
    att_error = np.zeros(n, dtype=float)

    for k in range(n):
        H = run["H"][k]
        Hd = run["Hdesired"][k]
        pos_error[k] = H[0:3, 3] - Hd[0:3, 3]
        Re = Hd[0:3, 0:3].T @ H[0:3, 0:3]
        cos_theta = np.clip((np.trace(Re) - 1.0) / 2.0, -1.0, 1.0)
        att_error[k] = np.arccos(cos_theta)

    vel_error = np.empty_like(np.asarray(run["V"], dtype=float))
    for k in range(n):
        He = inv_se3(run["Hdesired"][k]) @ run["H"][k]
        transported = adjoint_se3(inv_se3(He)) @ run["Vdesired"][k]
        vel_error[k] = run["V"][k] - transported

    wrench = np.asarray(run["wrench"], dtype=float)
    force_norm = np.linalg.norm(wrench[:, 3:6], axis=1)
    torque_norm = np.linalg.norm(wrench[:, 0:3], axis=1)

    metrics = {
        "positionRMSE": float(np.sqrt(np.mean(np.sum(pos_error**2, axis=1)))),
        "attitudeRMSE": float(np.sqrt(np.mean(att_error**2))),
        "angularVelocityRMSE": float(np.sqrt(np.mean(np.sum(vel_error[:, 0:3]**2, axis=1)))),
        "linearVelocityRMSE": float(np.sqrt(np.mean(np.sum(vel_error[:, 3:6]**2, axis=1)))),
        "maxPositionError": float(np.max(np.linalg.norm(pos_error, axis=1))),
        "forceRMS": float(np.sqrt(np.mean(force_norm**2))),
        "torqueRMS": float(np.sqrt(np.mean(torque_norm**2))),
    }

    metrics["finalSlidingNorm"] = float(np.linalg.norm(run["s"][-1]))
    metrics["maxPsi"] = float(np.max(run["Psi"]))
    metrics["maxVs"] = float(np.max(run["Vs"]))

    return metrics
