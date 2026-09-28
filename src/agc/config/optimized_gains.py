"""Optimized per-scenario gain registry.
Migrated seed gains: 2026-09-28 (scores cleared for the corrected objective)
"""

import numpy as np


base_tracking = {
    "KRdiag": np.array([0.002629, 0.002944, 0.1783]),
    "Kxidiag": np.array([32.3, 0.1116, 13.85]),
    "LambdaDiag": np.array([8.936, 11.53, 3.649, 0.09005, 0.1241, 0.1542]),
    "kd": 22.1,
    "ks": 0.4749,
    "alpha": 0.947,
}

adaptive_base_tracking = {
    "KRdiag": np.array([0.001117, 0.006384, 0.002431]),
    "Kxidiag": np.array([20.2, 0.4516, 12.84]),
    "LambdaDiag": np.array([7.96, 16.76, 2.417, 0.08501, 0.08359, 0.04791]),
    "kd": 14.44,
    "ks": 1.261,
    "alpha": 0.8361,
}

adaptive_base_entry = {
    **adaptive_base_tracking,
    "gammaE": 0.001 * np.ones(10),
    "gammaB": 0.001,
    "optimizationCost": None,
    "optimizationMetadata": None,
}

nominal_entry = {
    **base_tracking,
    "gammaE": 0.001 * np.ones(10),
    "gammaB": 0.001,
    "optimizationCost": None,
    "optimizationMetadata": None,
}

euclidean_entry = {
    **adaptive_base_tracking,
    "gammaB": 0.001,
    "gammaE": np.array([1e-05, 3.8e-05, 3.3e-05, 2.8e-05, 1.2e-05, 2.6e-05, 3e-05, 1.1e-05, 3.9e-05, 5.8e-05]),
    "optimizationCost": None,
    "optimizationMetadata": None,
}

bregman_entry = {
    **adaptive_base_tracking,
    "gammaE": 0.001 * np.ones(10),
    "gammaB": 0.1,
    "optimizationCost": None,
    "optimizationMetadata": None,
}


def optimized_gains(mode: str, coriolis: str = "lc") -> dict:
    """Return gains for nominal or adaptive control, invariant to Coriolis form."""
    m = mode.lower()
    c = coriolis.lower() if coriolis else "lc"
    key = f"{m}_{c}"
    registry = {
        "nominal_lc": nominal_entry,
        "nominal_rb": nominal_entry,
        "nominal": nominal_entry,
        "adaptive_base_lc": adaptive_base_entry,
        "adaptive_base_rb": adaptive_base_entry,
        "adaptive_base": adaptive_base_entry,
        "euclidean_lc": euclidean_entry,
        "euclidean_rb": euclidean_entry,
        "euclidean": euclidean_entry,
        "bregman_lc": bregman_entry,
        "bregman_rb": bregman_entry,
        "bregman": bregman_entry,
    }
    if key in registry:
        return registry[key]
    if m in registry:
        return registry[m]
    raise ValueError(f"Unknown mode/coriolis combination: {mode}/{coriolis}")
