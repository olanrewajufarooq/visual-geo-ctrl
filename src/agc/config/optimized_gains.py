"""Optimized per-scenario gain registry."""

import numpy as np


def optimized_gains(mode: str, coriolis: str) -> dict:
    """Return tuned per-scenario gains for each controller variant."""
    key = f"{mode.lower()}_{coriolis.lower()}"
    registry = {
        "nominal_c1": {
            "KRdiag": np.array([2.006, 0.3173, 3.523]),
            "Kxidiag": np.array([25.06, 6.507, 7.744]),
            "LambdaDiag": np.array([7.061, 8.789, 2.883, 0.1834, 3.764, 2.272]),
            "kd": 0.7041,
            "ks": 13.84,
            "alpha": 0.8554,
            "gammaE": 0.001 * np.ones(10),
            "gammaB": 0.001,
        },
        "nominal_c2": {
            "KRdiag": np.array([0.1819, 0.01, 2.596]),
            "Kxidiag": np.array([85.36, 3.784, 100.0]),
            "LambdaDiag": np.array([38.73, 26.23, 5.068, 0.1638, 0.2499, 0.1605]),
            "kd": 24.6,
            "ks": 1.651,
            "alpha": 0.1439,
            "gammaE": 0.001 * np.ones(10),
            "gammaB": 0.001,
        },
        "euclidean_c1": {
            "KRdiag": np.array([0.01926, 0.7074, 0.7917]),
            "Kxidiag": np.array([3.283, 69.9, 5.843]),
            "LambdaDiag": np.array([20.49, 16.85, 81.68, 0.3738, 0.5646, 0.4835]),
            "kd": 6.38,
            "ks": 13.01,
            "alpha": 0.5104,
            "gammaE": np.array([1.0, 0.3032, 0.2151, 0.1401, 1e-05, 1.046e-05, 0.03055, 1.319e-05, 0.001107, 0.001085]),
            "gammaB": 0.001,
        },
        "euclidean_c2": {
            "KRdiag": np.array([4.0, 5.0, 6.0]),
            "Kxidiag": np.array([3.0, 3.0, 4.0]),
            "LambdaDiag": np.array([2.0, 2.0, 2.0, 2.0, 2.0, 2.0]),
            "kd": 1.0,
            "ks": 0.5,
            "alpha": 0.5,
            "gammaE": 0.001 * np.ones(10),
            "gammaB": 0.001,
        },
        "bregman_c1": {
            "KRdiag": np.array([0.9401, 0.01072, 1.282]),
            "Kxidiag": np.array([11.31, 3.765, 5.802]),
            "LambdaDiag": np.array([18.28, 18.77, 14.41, 0.7082, 0.3636, 0.7342]),
            "kd": 6.108,
            "ks": 12.93,
            "alpha": 0.4534,
            "gammaE": 0.001 * np.ones(10),
            "gammaB": 0.09003,
        },
        "bregman_c2": {
            "KRdiag": np.array([0.01887, 0.00132, 0.001561]),
            "Kxidiag": np.array([11.71, 11.34, 299.5]),
            "LambdaDiag": np.array([18.56, 34.92, 11.97, 0.08278, 0.1263, 0.4643]),
            "kd": 16.66,
            "ks": 0.001917,
            "alpha": 0.1386,
            "gammaE": 0.001 * np.ones(10),
            "gammaB": 0.08016,
        },
    }

    if key not in registry:
        raise KeyError(f"Unknown scenario key: '{key}'.")
    return registry[key]
