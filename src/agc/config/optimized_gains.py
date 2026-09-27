"""Optimized per-scenario gain registry.
Last promotion: 2026-09-27 06:49:25 (Stage: adaptive, Cost: 0.398043)
"""

import numpy as np


def optimized_gains(mode: str, coriolis: str = "lc") -> dict:
    """Return tuned gains for each controller variant (invariant to Coriolis)."""
    m = mode.lower()
    c = coriolis.lower() if coriolis else "lc"
    key = f"{m}_{c}"
    registry = {
        "nominal_lc": {
            "KRdiag": np.array([np.float64(8.737), np.float64(12.15), np.float64(2.74)]),
            "Kxidiag": np.array([np.float64(67.45), np.float64(97.26), np.float64(92.51)]),
            "LambdaDiag": np.array([np.float64(5.033), np.float64(2.628), np.float64(5.99), np.float64(0.2608), np.float64(0.148), np.float64(0.2848)]),
            "kd": 3.067,
            "ks": 2.391,
            "alpha": 0.7739,
            "gammaE": np.array([np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001)]),
            "gammaB": 1e-06,
            "optimizationCost": 0.398042616711,
        },
        "nominal_rb": {
            "KRdiag": np.array([np.float64(8.737), np.float64(12.15), np.float64(2.74)]),
            "Kxidiag": np.array([np.float64(67.45), np.float64(97.26), np.float64(92.51)]),
            "LambdaDiag": np.array([np.float64(5.033), np.float64(2.628), np.float64(5.99), np.float64(0.2608), np.float64(0.148), np.float64(0.2848)]),
            "kd": 3.067,
            "ks": 2.391,
            "alpha": 0.7739,
            "gammaE": np.array([np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001)]),
            "gammaB": 1e-06,
            "optimizationCost": 0.398042616711,
        },
        "euclidean_lc": {
            "KRdiag": np.array([np.float64(8.737), np.float64(12.15), np.float64(2.74)]),
            "Kxidiag": np.array([np.float64(67.45), np.float64(97.26), np.float64(92.51)]),
            "LambdaDiag": np.array([np.float64(5.033), np.float64(2.628), np.float64(5.99), np.float64(0.2608), np.float64(0.148), np.float64(0.2848)]),
            "kd": 3.067,
            "ks": 2.391,
            "alpha": 0.7739,
            "gammaE": np.array([np.float64(1e-05), np.float64(4.8e-05), np.float64(3e-05), np.float64(1.2e-05), np.float64(9.9e-05), np.float64(1.2e-05), np.float64(7.4e-05), np.float64(1e-05), np.float64(1.3e-05), np.float64(0.0002)]),
            "gammaB": 1e-06,
            "optimizationCost": 22160,
        },
        "euclidean_rb": {
            "KRdiag": np.array([np.float64(8.737), np.float64(12.15), np.float64(2.74)]),
            "Kxidiag": np.array([np.float64(67.45), np.float64(97.26), np.float64(92.51)]),
            "LambdaDiag": np.array([np.float64(5.033), np.float64(2.628), np.float64(5.99), np.float64(0.2608), np.float64(0.148), np.float64(0.2848)]),
            "kd": 3.067,
            "ks": 2.391,
            "alpha": 0.7739,
            "gammaE": np.array([np.float64(1e-05), np.float64(4.8e-05), np.float64(3e-05), np.float64(1.2e-05), np.float64(9.9e-05), np.float64(1.2e-05), np.float64(7.4e-05), np.float64(1e-05), np.float64(1.3e-05), np.float64(0.0002)]),
            "gammaB": 1e-06,
            "optimizationCost": 22160,
        },
        "bregman_lc": {
            "KRdiag": np.array([np.float64(8.737), np.float64(12.15), np.float64(2.74)]),
            "Kxidiag": np.array([np.float64(67.45), np.float64(97.26), np.float64(92.51)]),
            "LambdaDiag": np.array([np.float64(5.033), np.float64(2.628), np.float64(5.99), np.float64(0.2608), np.float64(0.148), np.float64(0.2848)]),
            "kd": 3.067,
            "ks": 2.391,
            "alpha": 0.7739,
            "gammaE": np.array([np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001)]),
            "gammaB": 2e-06,
            "optimizationCost": 0.398042606196,
        },
        "bregman_rb": {
            "KRdiag": np.array([np.float64(8.737), np.float64(12.15), np.float64(2.74)]),
            "Kxidiag": np.array([np.float64(67.45), np.float64(97.26), np.float64(92.51)]),
            "LambdaDiag": np.array([np.float64(5.033), np.float64(2.628), np.float64(5.99), np.float64(0.2608), np.float64(0.148), np.float64(0.2848)]),
            "kd": 3.067,
            "ks": 2.391,
            "alpha": 0.7739,
            "gammaE": np.array([np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001)]),
            "gammaB": 2e-06,
            "optimizationCost": 0.398042606196,
        },
        "adaptive_lc": {
            "KRdiag": np.array([np.float64(8.737), np.float64(12.15), np.float64(2.74)]),
            "Kxidiag": np.array([np.float64(67.45), np.float64(97.26), np.float64(92.51)]),
            "LambdaDiag": np.array([np.float64(5.033), np.float64(2.628), np.float64(5.99), np.float64(0.2608), np.float64(0.148), np.float64(0.2848)]),
            "kd": 3.067,
            "ks": 2.391,
            "alpha": 0.7739,
            "gammaE": np.array([np.float64(1e-05), np.float64(4.8e-05), np.float64(3e-05), np.float64(1.2e-05), np.float64(9.9e-05), np.float64(1.2e-05), np.float64(7.4e-05), np.float64(1e-05), np.float64(1.3e-05), np.float64(0.0002)]),
            "gammaB": 1e-06,
            "optimizationCost": 22160,
        },
        "adaptive_rb": {
            "KRdiag": np.array([np.float64(8.737), np.float64(12.15), np.float64(2.74)]),
            "Kxidiag": np.array([np.float64(67.45), np.float64(97.26), np.float64(92.51)]),
            "LambdaDiag": np.array([np.float64(5.033), np.float64(2.628), np.float64(5.99), np.float64(0.2608), np.float64(0.148), np.float64(0.2848)]),
            "kd": 3.067,
            "ks": 2.391,
            "alpha": 0.7739,
            "gammaE": np.array([np.float64(1e-05), np.float64(4.8e-05), np.float64(3e-05), np.float64(1.2e-05), np.float64(9.9e-05), np.float64(1.2e-05), np.float64(7.4e-05), np.float64(1e-05), np.float64(1.3e-05), np.float64(0.0002)]),
            "gammaB": 1e-06,
            "optimizationCost": 22160,
        },
        "adaptive": {
            "KRdiag": np.array([np.float64(8.737), np.float64(12.15), np.float64(2.74)]),
            "Kxidiag": np.array([np.float64(67.45), np.float64(97.26), np.float64(92.51)]),
            "LambdaDiag": np.array([np.float64(5.033), np.float64(2.628), np.float64(5.99), np.float64(0.2608), np.float64(0.148), np.float64(0.2848)]),
            "kd": 3.067,
            "ks": 2.391,
            "alpha": 0.7739,
            "gammaE": np.array([np.float64(1e-05), np.float64(4.8e-05), np.float64(3e-05), np.float64(1.2e-05), np.float64(9.9e-05), np.float64(1.2e-05), np.float64(7.4e-05), np.float64(1e-05), np.float64(1.3e-05), np.float64(0.0002)]),
            "gammaB": 1e-06,
            "optimizationCost": 22160,
        },
        "nominal": {
            "KRdiag": np.array([np.float64(8.737), np.float64(12.15), np.float64(2.74)]),
            "Kxidiag": np.array([np.float64(67.45), np.float64(97.26), np.float64(92.51)]),
            "LambdaDiag": np.array([np.float64(5.033), np.float64(2.628), np.float64(5.99), np.float64(0.2608), np.float64(0.148), np.float64(0.2848)]),
            "kd": 3.067,
            "ks": 2.391,
            "alpha": 0.7739,
            "gammaE": np.array([np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001)]),
            "gammaB": 1e-06,
            "optimizationCost": 0.398042616711,
        },
        "euclidean": {
            "KRdiag": np.array([np.float64(8.737), np.float64(12.15), np.float64(2.74)]),
            "Kxidiag": np.array([np.float64(67.45), np.float64(97.26), np.float64(92.51)]),
            "LambdaDiag": np.array([np.float64(5.033), np.float64(2.628), np.float64(5.99), np.float64(0.2608), np.float64(0.148), np.float64(0.2848)]),
            "kd": 3.067,
            "ks": 2.391,
            "alpha": 0.7739,
            "gammaE": np.array([np.float64(1e-05), np.float64(4.8e-05), np.float64(3e-05), np.float64(1.2e-05), np.float64(9.9e-05), np.float64(1.2e-05), np.float64(7.4e-05), np.float64(1e-05), np.float64(1.3e-05), np.float64(0.0002)]),
            "gammaB": 1e-06,
            "optimizationCost": 22160,
        },
        "bregman": {
            "KRdiag": np.array([np.float64(8.737), np.float64(12.15), np.float64(2.74)]),
            "Kxidiag": np.array([np.float64(67.45), np.float64(97.26), np.float64(92.51)]),
            "LambdaDiag": np.array([np.float64(5.033), np.float64(2.628), np.float64(5.99), np.float64(0.2608), np.float64(0.148), np.float64(0.2848)]),
            "kd": 3.067,
            "ks": 2.391,
            "alpha": 0.7739,
            "gammaE": np.array([np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001)]),
            "gammaB": 2e-06,
            "optimizationCost": 0.398042606196,
        },
    }
    if key in registry:
        return registry[key]
    if m in registry:
        return registry[m]
    raise ValueError(f"Unknown mode/coriolis combination: {mode}/{coriolis}")
