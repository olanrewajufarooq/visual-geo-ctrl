"""Optimized per-scenario gain registry.
Last promotion: 2026-09-22 08:40:08 (Stage: all, Cost: 90175.3)
"""

import numpy as np


def optimized_gains(mode: str, coriolis: str) -> dict:
    """Return tuned per-scenario gains for each controller variant."""
    key = f"{mode.lower()}_{coriolis.lower()}"
    registry = {
        "nominal_c1": {
            "KRdiag": np.array([7.621, 7.179, 1.906]),
            "Kxidiag": np.array([61.84, 75.45, 61.86]),
            "LambdaDiag": np.array([11.23, 9.577, 6.622, 0.147, 0.1825, 0.1369]),
            "kd": 1.612,
            "ks": 29.86,
            "alpha": 0.8654,
            "gammaE": np.array([0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001]),
            "gammaB": 0.001,
        },
        "nominal_c2": {
            "KRdiag": np.array([0.1819, 0.01, 2.596]),
            "Kxidiag": np.array([85.36, 3.784, 100.0]),
            "LambdaDiag": np.array([38.73, 26.23, 5.068, 0.1638, 0.2499, 0.1605]),
            "kd": 24.6,
            "ks": 1.651,
            "alpha": 0.1439,
            "gammaE": np.array([0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001]),
            "gammaB": 0.001,
        },
        "euclidean_c1": {
            "KRdiag": np.array([0.01926, 0.7074, 0.7917]),
            "Kxidiag": np.array([3.283, 69.9, 5.843]),
            "LambdaDiag": np.array([20.49, 16.85, 81.68, 0.3738, 0.5646, 0.4835]),
            "kd": 6.38,
            "ks": 13.01,
            "alpha": 0.5104,
            "gammaE": np.array([1.0, 0.3032, 0.2151, 0.1401, 1e-05, 1e-05, 0.03055, 1.3e-05, 0.001107, 0.001085]),
            "gammaB": 0.001,
        },
        "euclidean_c2": {
            "KRdiag": np.array([0.001003, 0.02792, 0.06612]),
            "Kxidiag": np.array([0.04397, 0.168, 0.2117]),
            "LambdaDiag": np.array([0.9909, 3.077, 9.063, 0.02129, 5.775, 0.02663]),
            "kd": 4.331,
            "ks": 5.188,
            "alpha": 0.7088,
            "gammaE": np.array([0.000164, 2e-05, 1.6e-05, 5.6e-05, 0.000329, 0.000117, 0.004221, 0.001884, 1.2e-05, 0.000286]),
            "gammaB": 0.001,
        },
        "bregman_c1": {
            "KRdiag": np.array([0.9401, 0.01072, 1.282]),
            "Kxidiag": np.array([11.31, 3.765, 5.802]),
            "LambdaDiag": np.array([18.28, 18.77, 14.41, 0.7082, 0.3636, 0.7342]),
            "kd": 6.108,
            "ks": 12.93,
            "alpha": 0.4534,
            "gammaE": np.array([0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001]),
            "gammaB": 0.09003,
        },
        "bregman_c2": {
            "KRdiag": np.array([0.01887, 0.00132, 0.001561]),
            "Kxidiag": np.array([11.71, 11.34, 299.5]),
            "LambdaDiag": np.array([18.56, 34.92, 11.97, 0.08278, 0.1263, 0.4643]),
            "kd": 16.66,
            "ks": 0.001917,
            "alpha": 0.1386,
            "gammaE": np.array([0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001]),
            "gammaB": 0.08016,
        },
    }
    if key not in registry:
        raise ValueError(f"Unknown mode/coriolis combination: {mode}/{coriolis}")
    return registry[key]
