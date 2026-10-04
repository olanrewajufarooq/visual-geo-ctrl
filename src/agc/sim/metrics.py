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

    if "activePlantPi" in run and "estimatePi" in run:
        true_pi = run["activePlantPi"]
        est_pi = run["estimatePi"]

        mass_scale = max(abs(true_pi[0, 0]), 1.0)
        mass_error = (est_pi[:, 0] - true_pi[:, 0]) / mass_scale

        true_mass = np.maximum(np.abs(true_pi[:, 0:1]), 1e-12)
        est_mass = np.maximum(np.abs(est_pi[:, 0:1]), 1e-12)

        true_cog = true_pi[:, 1:4] / true_mass
        est_cog = est_pi[:, 1:4] / est_mass

        cog_scale = max(np.linalg.norm(true_cog[0]), 0.1)
        cog_error = np.linalg.norm(est_cog - true_cog, axis=1) / cog_scale

        inertia_scale = max(np.linalg.norm(true_pi[0, 4:10]), 1.0)
        inertia_error = np.linalg.norm(est_pi[:, 4:10] - true_pi[:, 4:10], axis=1) / inertia_scale

        metrics["massEstimationRMSE"] = float(np.sqrt(np.mean(mass_error**2)))
        metrics["centerOfMassEstimationRMSE"] = float(np.sqrt(np.mean(cog_error**2)))
        metrics["massCogEstimationRMSE"] = float(np.sqrt(np.mean(0.5 * (mass_error**2 + cog_error**2))))
        metrics["inertiaEstimationRMSE"] = float(np.sqrt(np.mean(inertia_error**2)))

        param_scale = max(np.linalg.norm(true_pi[0]), 1.0)
        param_error = np.linalg.norm(est_pi - true_pi, axis=1) / param_scale
        metrics["parameterEstimationRMSE"] = float(np.sqrt(np.mean(param_error**2)))
    else:
        metrics["massEstimationRMSE"] = float("nan")
        metrics["centerOfMassEstimationRMSE"] = float("nan")
        metrics["massCogEstimationRMSE"] = float("nan")
        metrics["inertiaEstimationRMSE"] = float("nan")
        metrics["parameterEstimationRMSE"] = float("nan")

    metrics["finalSlidingNorm"] = float(np.linalg.norm(run["s"][-1]))
    metrics["maxPsi"] = float(np.max(run["Psi"]))
    metrics["maxVs"] = float(np.max(run["Vs"]))

    min_pseudo = run["minPseudoEigenvalue"]
    valid_min = min_pseudo[~np.isnan(min_pseudo)]
    metrics["minimumPseudoEigenvalue"] = float(np.min(valid_min)) if len(valid_min) > 0 else float("nan")

    return metrics
