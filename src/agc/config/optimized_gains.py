"""Optimized per-scenario gain registry.
Last promotion: 2026-09-28 00:25:08 (Stage: all, Cost: 27757.6)
"""

import numpy as np


def optimized_gains(mode: str, coriolis: str = "lc") -> dict:
    """Return tuned gains for each controller variant (invariant to Coriolis)."""
    m = mode.lower()
    c = coriolis.lower() if coriolis else "lc"
    key = f"{m}_{c}"
    registry = {
        "nominal_lc": {
            "KRdiag": np.array([np.float64(0.001117), np.float64(0.006384), np.float64(0.002431)]),
            "Kxidiag": np.array([np.float64(20.2), np.float64(0.4516), np.float64(12.84)]),
            "LambdaDiag": np.array([np.float64(7.96), np.float64(16.76), np.float64(2.417), np.float64(0.08501), np.float64(0.08359), np.float64(0.04791)]),
            "kd": 14.44,
            "ks": 1.261,
            "alpha": 0.8361,
            "gammaE": np.array([np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001)]),
            "gammaB": 0.001,
            "optimizationCost": 27757.5574693,
        },
        "nominal_rb": {
            "KRdiag": np.array([np.float64(0.001117), np.float64(0.006384), np.float64(0.002431)]),
            "Kxidiag": np.array([np.float64(20.2), np.float64(0.4516), np.float64(12.84)]),
            "LambdaDiag": np.array([np.float64(7.96), np.float64(16.76), np.float64(2.417), np.float64(0.08501), np.float64(0.08359), np.float64(0.04791)]),
            "kd": 14.44,
            "ks": 1.261,
            "alpha": 0.8361,
            "gammaE": np.array([np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001)]),
            "gammaB": 0.001,
            "optimizationCost": 27757.5574693,
        },
        "euclidean_lc": {
            "KRdiag": np.array([np.float64(0.001117), np.float64(0.006384), np.float64(0.002431)]),
            "Kxidiag": np.array([np.float64(20.2), np.float64(0.4516), np.float64(12.84)]),
            "LambdaDiag": np.array([np.float64(7.96), np.float64(16.76), np.float64(2.417), np.float64(0.08501), np.float64(0.08359), np.float64(0.04791)]),
            "kd": 14.44,
            "ks": 1.261,
            "alpha": 0.8361,
            "gammaE": np.array([np.float64(0.06224), np.float64(0.04911), np.float64(0.003038), np.float64(0.01321), np.float64(0.08969), np.float64(0.004635), np.float64(0.009389), np.float64(0.000774), np.float64(6e-05), np.float64(0.000156)]),
            "gammaB": 0.001,
            "optimizationCost": None,
            "optimizationMetadata": None,
        },
        "euclidean_rb": {
            "KRdiag": np.array([np.float64(0.001117), np.float64(0.006384), np.float64(0.002431)]),
            "Kxidiag": np.array([np.float64(20.2), np.float64(0.4516), np.float64(12.84)]),
            "LambdaDiag": np.array([np.float64(7.96), np.float64(16.76), np.float64(2.417), np.float64(0.08501), np.float64(0.08359), np.float64(0.04791)]),
            "kd": 14.44,
            "ks": 1.261,
            "alpha": 0.8361,
            "gammaE": np.array([np.float64(0.06224), np.float64(0.04911), np.float64(0.003038), np.float64(0.01321), np.float64(0.08969), np.float64(0.004635), np.float64(0.009389), np.float64(0.000774), np.float64(6e-05), np.float64(0.000156)]),
            "gammaB": 0.001,
            "optimizationCost": None,
            "optimizationMetadata": None,
        },
        "bregman_lc": {
            "KRdiag": np.array([np.float64(0.001117), np.float64(0.006384), np.float64(0.002431)]),
            "Kxidiag": np.array([np.float64(20.2), np.float64(0.4516), np.float64(12.84)]),
            "LambdaDiag": np.array([np.float64(7.96), np.float64(16.76), np.float64(2.417), np.float64(0.08501), np.float64(0.08359), np.float64(0.04791)]),
            "kd": 14.44,
            "ks": 1.261,
            "alpha": 0.8361,
            "gammaE": np.array([np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001)]),
            "gammaB": 0.09054,
            "optimizationCost": None,
            "optimizationMetadata": None,
        },
        "bregman_rb": {
            "KRdiag": np.array([np.float64(0.001117), np.float64(0.006384), np.float64(0.002431)]),
            "Kxidiag": np.array([np.float64(20.2), np.float64(0.4516), np.float64(12.84)]),
            "LambdaDiag": np.array([np.float64(7.96), np.float64(16.76), np.float64(2.417), np.float64(0.08501), np.float64(0.08359), np.float64(0.04791)]),
            "kd": 14.44,
            "ks": 1.261,
            "alpha": 0.8361,
            "gammaE": np.array([np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001)]),
            "gammaB": 0.09054,
            "optimizationCost": None,
            "optimizationMetadata": None,
        },
        "nominal": {
            "KRdiag": np.array([np.float64(0.001117), np.float64(0.006384), np.float64(0.002431)]),
            "Kxidiag": np.array([np.float64(20.2), np.float64(0.4516), np.float64(12.84)]),
            "LambdaDiag": np.array([np.float64(7.96), np.float64(16.76), np.float64(2.417), np.float64(0.08501), np.float64(0.08359), np.float64(0.04791)]),
            "kd": 14.44,
            "ks": 1.261,
            "alpha": 0.8361,
            "gammaE": np.array([np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001)]),
            "gammaB": 0.001,
            "optimizationCost": 27757.5574693,
        },
        "euclidean": {
            "KRdiag": np.array([np.float64(0.001117), np.float64(0.006384), np.float64(0.002431)]),
            "Kxidiag": np.array([np.float64(20.2), np.float64(0.4516), np.float64(12.84)]),
            "LambdaDiag": np.array([np.float64(7.96), np.float64(16.76), np.float64(2.417), np.float64(0.08501), np.float64(0.08359), np.float64(0.04791)]),
            "kd": 14.44,
            "ks": 1.261,
            "alpha": 0.8361,
            "gammaE": np.array([np.float64(0.06224), np.float64(0.04911), np.float64(0.003038), np.float64(0.01321), np.float64(0.08969), np.float64(0.004635), np.float64(0.009389), np.float64(0.000774), np.float64(6e-05), np.float64(0.000156)]),
            "gammaB": 0.001,
            "optimizationCost": None,
            "optimizationMetadata": None,
        },
        "bregman": {
            "KRdiag": np.array([np.float64(0.001117), np.float64(0.006384), np.float64(0.002431)]),
            "Kxidiag": np.array([np.float64(20.2), np.float64(0.4516), np.float64(12.84)]),
            "LambdaDiag": np.array([np.float64(7.96), np.float64(16.76), np.float64(2.417), np.float64(0.08501), np.float64(0.08359), np.float64(0.04791)]),
            "kd": 14.44,
            "ks": 1.261,
            "alpha": 0.8361,
            "gammaE": np.array([np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001)]),
            "gammaB": 0.09054,
            "optimizationCost": None,
            "optimizationMetadata": None,
        },
    }
    if key in registry:
        return registry[key]
    if m in registry:
        return registry[m]
    raise ValueError(f"Unknown mode/coriolis combination: {mode}/{coriolis}")
