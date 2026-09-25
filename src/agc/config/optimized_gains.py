"""Optimized per-scenario gain registry.
Last promotion: 2026-09-25 (Synchronized with results/optimization/best-gain records)
"""

import numpy as np


def optimized_gains(mode: str, coriolis: str) -> dict:
    """Return tuned per-scenario gains for each controller variant."""
    key = f"{mode.lower()}_{coriolis.lower()}"
    registry = {
        "nominal_lc": {
            "KRdiag": np.array([8.737, 12.15, 2.74]),
            "Kxidiag": np.array([67.45, 97.26, 92.51]),
            "LambdaDiag": np.array([6.979, 6.767, 4.755, 0.1, 0.1278, 0.1136]),
            "kd": 0.5913,
            "ks": 22.97,
            "alpha": 0.8929,
            "gammaE": np.array([0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001]),
            "gammaB": 0.001,
            "optimizationCost": 2800.3647653189714,
        },
        "nominal_rb": {
            "KRdiag": np.array([3.718, 3.355, 2.812]),
            "Kxidiag": np.array([51.77, 22.57, 12.15]),
            "LambdaDiag": np.array([1.078, 1.088, 1.424, 0.2863, 0.5781, 0.2203]),
            "kd": 5.111,
            "ks": 3.23,
            "alpha": 0.95,
            "gammaE": np.array([0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001]),
            "gammaB": 0.001,
            "optimizationCost": 2836.7810230005257,
        },
        "euclidean_lc": {
            "KRdiag": np.array([2.507, 0.004643, 2.368]),
            "Kxidiag": np.array([2.843, 4.466, 6.498]),
            "LambdaDiag": np.array([1.032, 4.493, 1.707, 0.02924, 5.154, 0.0798]),
            "kd": 0.9206,
            "ks": 1.728,
            "alpha": 0.5898,
            "gammaE": np.array([0.06224, 0.04911, 0.003038, 0.01321, 0.08969, 0.004635, 0.009389, 0.000774, 5.955e-05, 0.0001565]),
            "gammaB": 0.001,
            "optimizationCost": 79.75069430898473,
        },
        "euclidean_rb": {
            "KRdiag": np.array([0.005718, 0.001965, 0.2317]),
            "Kxidiag": np.array([27.91, 72.81, 8.581]),
            "LambdaDiag": np.array([1.148, 1.368, 1.896, 0.2372, 0.261, 0.2487]),
            "kd": 0.9262,
            "ks": 8.475,
            "alpha": 0.9486,
            "gammaE": np.array([0.9417, 0.08606, 0.02651, 0.05273, 0.0002806, 0.000113, 0.3724, 0.001736, 0.01859, 0.005186]),
            "gammaB": 0.001,
            "optimizationCost": 80.16094054975478,
        },
        "bregman_lc": {
            "KRdiag": np.array([0.1784, 6.418, 0.4457]),
            "Kxidiag": np.array([9.833, 1.762, 33.96]),
            "LambdaDiag": np.array([4.392, 19.97, 15.26, 0.7227, 0.967, 0.1144]),
            "kd": 7.934,
            "ks": 0.4086,
            "alpha": 0.5731,
            "gammaE": np.array([0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001]),
            "gammaB": 0.09054,
            "optimizationCost": 61.35170658876754,
        },
        "bregman_rb": {
            "KRdiag": np.array([0.006092, 0.1423, 0.01086]),
            "Kxidiag": np.array([7.802, 8.541, 1.926]),
            "LambdaDiag": np.array([1.284, 1.088, 0.8828, 0.3279, 0.3048, 0.04811]),
            "kd": 7.842,
            "ks": 1.109,
            "alpha": 0.5363,
            "gammaE": np.array([0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001]),
            "gammaB": 0.09904,
            "optimizationCost": 284.21366333293787,
        },
    }
    if key not in registry:
        raise ValueError(f"Unknown mode/coriolis combination: {mode}/{coriolis}")
    return registry[key]
