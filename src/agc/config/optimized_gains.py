"""Optimized per-scenario gain registry.
Last promotion: 2026-09-25 (Synchronized with results/optimization/best-gain records)
"""

import numpy as np


def optimized_gains(mode: str, coriolis: str = "lc") -> dict:
    """Return tuned gains for each controller variant (invariant to Coriolis)."""
    m = mode.lower()
    c = coriolis.lower() if coriolis else "lc"
    key = f"{m}_{c}"

    # Shared base tracking and sliding surface gains
    base_tracking = {
        "KRdiag": np.array([0.1784, 6.418, 0.4457]),
        "Kxidiag": np.array([9.833, 1.762, 33.96]),
        "LambdaDiag": np.array([4.392, 19.97, 15.26, 0.7227, 0.967, 0.1144]),
        "kd": 7.934,
        "ks": 0.4086,
        "alpha": 0.5731,
    }

    nominal_entry = {
        **base_tracking,
        "gammaE": 0.001 * np.ones(10),
        "gammaB": 0.001,
        "optimizationCost": 61.3517,
    }

    euclidean_entry = {
        **base_tracking,
        "gammaE": np.array([0.06224, 0.04911, 0.003038, 0.01321, 0.08969, 0.004635, 0.009389, 0.000774, 5.955e-05, 0.0001565]),
        "gammaB": 0.001,
        "optimizationCost": 79.7507,
    }

    bregman_entry = {
        **base_tracking,
        "gammaE": 0.001 * np.ones(10),
        "gammaB": 0.09054,
        "optimizationCost": 61.3517,
    }

    registry = {
        "nominal_lc": nominal_entry,
        "nominal_rb": nominal_entry,
        "nominal": nominal_entry,
        "euclidean_lc": euclidean_entry,
        "euclidean_rb": euclidean_entry,
        "euclidean": euclidean_entry,
        "bregman_lc": bregman_entry,
        "bregman_rb": bregman_entry,
        "bregman": bregman_entry,
        "adaptive_lc": euclidean_entry,
        "adaptive_rb": euclidean_entry,
        "adaptive": euclidean_entry,
    }

    if key in registry:
        return registry[key]
    if m in registry:
        return registry[m]
    raise ValueError(f"Unknown mode/coriolis combination: {mode}/{coriolis}")
