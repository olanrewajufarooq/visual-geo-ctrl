"""Optimized per-scenario gain registry.
Last promotion: 2026-09-29 18:47:14 (Stage: tracking, Cost: 189.825)
"""

import numpy as np


# Shared nominal tracking and sliding-surface gains.
base_tracking = {
    "KRdiag": np.array([0.03352, 0.0217, 0.003367]),
    "Kxidiag": np.array([31.96, 25.52, 8.649]),
    "LambdaDiag": np.array([46.79, 29.15, 21.13, 0.09285, 0.1236, 0.06351]),
    "kd": 14.14,
    "ks": 10.31,
    "alpha": 0.8453,
}

# Shared tracking and sliding-surface gains for both adaptive estimators.
adaptive_base_tracking = {
    "KRdiag": np.array([0.001117, 0.006384, 0.002431]),
    "Kxidiag": np.array([20.2, 0.4516, 12.84]),
    "LambdaDiag": np.array([7.96, 16.76, 2.417, 0.08501, 0.08359, 0.04791]),
    "kd": 14.44,
    "ks": 1.261,
    "alpha": 0.8361,
}

nominal_entry = {
    **base_tracking,
    "gammaE": np.array([0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001]),
    "gammaB": 0.001,
    "optimizationCost": 189.824830929,
    "optimizationMetadata": {'cost': 189.82483092892653, 'stage': 'tracking', 'artifact': 'C:\\Users\\g202404900\\Desktop\\adaptive-geo-ctrl-pybullet\\.worktrees\\paper-drift-fixes\\results\\optimization\\best-gain\\nominal', 'contextHash': '593e2e812427efc88cef370a58c93cafb5adde2644a15502a229e2ace6463ce6', 'objectiveVersion': 'force-torque-separated-mean-v1'},
}

adaptive_base_entry = {
    **adaptive_base_tracking,
    "gammaE": np.array([0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001]),
    "gammaB": 0.001,
    "optimizationCost": None,
    "optimizationMetadata": None,
}

euclidean_entry = {
    **adaptive_base_tracking,
    "gammaE": np.array([1e-05, 3.8e-05, 3.3e-05, 2.8e-05, 1.2e-05, 2.6e-05, 3e-05, 1.1e-05, 3.9e-05, 5.8e-05]),
    "gammaB": 0.001,
    "optimizationCost": None,
    "optimizationMetadata": None,
}

bregman_entry = {
    **adaptive_base_tracking,
    "gammaE": np.array([0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001]),
    "gammaB": 0.1,
    "optimizationCost": None,
    "optimizationMetadata": None,
}

def optimized_gains(mode: str, coriolis: str = "lc") -> dict:
    """Return tuned gains for each controller variant (invariant to Coriolis)."""
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
