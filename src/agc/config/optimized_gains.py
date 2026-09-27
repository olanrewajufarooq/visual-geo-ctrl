"""Optimized per-scenario gain registry.
Last promotion: 2026-09-27 09:57:45 (Stage: all, Cost: 10618.7)
"""

import numpy as np


def optimized_gains(mode: str, coriolis: str = "lc") -> dict:
    """Return tuned gains for each controller variant (invariant to Coriolis)."""
    m = mode.lower()
    c = coriolis.lower() if coriolis else "lc"
    key = f"{m}_{c}"
    registry = {
        "nominal_lc": {
            "KRdiag": np.array([np.float64(0.002629), np.float64(0.002944), np.float64(0.1783)]),
            "Kxidiag": np.array([np.float64(32.3), np.float64(0.1116), np.float64(13.85)]),
            "LambdaDiag": np.array([np.float64(8.936), np.float64(11.53), np.float64(3.649), np.float64(0.09005), np.float64(0.1241), np.float64(0.1542)]),
            "kd": 22.1,
            "ks": 0.4749,
            "alpha": 0.947,
            "gammaE": np.array([np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001)]),
            "gammaB": 0.001,
            "optimizationCost": 10618.6606262,
        },
        "nominal_rb": {
            "KRdiag": np.array([np.float64(0.002629), np.float64(0.002944), np.float64(0.1783)]),
            "Kxidiag": np.array([np.float64(32.3), np.float64(0.1116), np.float64(13.85)]),
            "LambdaDiag": np.array([np.float64(8.936), np.float64(11.53), np.float64(3.649), np.float64(0.09005), np.float64(0.1241), np.float64(0.1542)]),
            "kd": 22.1,
            "ks": 0.4749,
            "alpha": 0.947,
            "gammaE": np.array([np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001)]),
            "gammaB": 0.001,
            "optimizationCost": 10618.6606262,
        },
        "euclidean_lc": {
            "KRdiag": np.array([np.float64(0.002629), np.float64(0.002944), np.float64(0.1783)]),
            "Kxidiag": np.array([np.float64(32.3), np.float64(0.1116), np.float64(13.85)]),
            "LambdaDiag": np.array([np.float64(8.936), np.float64(11.53), np.float64(3.649), np.float64(0.09005), np.float64(0.1241), np.float64(0.1542)]),
            "kd": 22.1,
            "ks": 0.4749,
            "alpha": 0.947,
            "gammaE": np.array([np.float64(0.06224), np.float64(0.04911), np.float64(0.003038), np.float64(0.01321), np.float64(0.08969), np.float64(0.004635), np.float64(0.009389), np.float64(0.000774), np.float64(6e-05), np.float64(0.000156)]),
            "gammaB": 0.001,
            "optimizationCost": 79.7507,
        },
        "euclidean_rb": {
            "KRdiag": np.array([np.float64(0.002629), np.float64(0.002944), np.float64(0.1783)]),
            "Kxidiag": np.array([np.float64(32.3), np.float64(0.1116), np.float64(13.85)]),
            "LambdaDiag": np.array([np.float64(8.936), np.float64(11.53), np.float64(3.649), np.float64(0.09005), np.float64(0.1241), np.float64(0.1542)]),
            "kd": 22.1,
            "ks": 0.4749,
            "alpha": 0.947,
            "gammaE": np.array([np.float64(0.06224), np.float64(0.04911), np.float64(0.003038), np.float64(0.01321), np.float64(0.08969), np.float64(0.004635), np.float64(0.009389), np.float64(0.000774), np.float64(6e-05), np.float64(0.000156)]),
            "gammaB": 0.001,
            "optimizationCost": 79.7507,
        },
        "bregman_lc": {
            "KRdiag": np.array([np.float64(0.002629), np.float64(0.002944), np.float64(0.1783)]),
            "Kxidiag": np.array([np.float64(32.3), np.float64(0.1116), np.float64(13.85)]),
            "LambdaDiag": np.array([np.float64(8.936), np.float64(11.53), np.float64(3.649), np.float64(0.09005), np.float64(0.1241), np.float64(0.1542)]),
            "kd": 22.1,
            "ks": 0.4749,
            "alpha": 0.947,
            "gammaE": np.array([np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001)]),
            "gammaB": 0.09054,
            "optimizationCost": 61.3517,
        },
        "bregman_rb": {
            "KRdiag": np.array([np.float64(0.002629), np.float64(0.002944), np.float64(0.1783)]),
            "Kxidiag": np.array([np.float64(32.3), np.float64(0.1116), np.float64(13.85)]),
            "LambdaDiag": np.array([np.float64(8.936), np.float64(11.53), np.float64(3.649), np.float64(0.09005), np.float64(0.1241), np.float64(0.1542)]),
            "kd": 22.1,
            "ks": 0.4749,
            "alpha": 0.947,
            "gammaE": np.array([np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001)]),
            "gammaB": 0.09054,
            "optimizationCost": 61.3517,
        },
        "nominal": {
            "KRdiag": np.array([np.float64(0.002629), np.float64(0.002944), np.float64(0.1783)]),
            "Kxidiag": np.array([np.float64(32.3), np.float64(0.1116), np.float64(13.85)]),
            "LambdaDiag": np.array([np.float64(8.936), np.float64(11.53), np.float64(3.649), np.float64(0.09005), np.float64(0.1241), np.float64(0.1542)]),
            "kd": 22.1,
            "ks": 0.4749,
            "alpha": 0.947,
            "gammaE": np.array([np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001)]),
            "gammaB": 0.001,
            "optimizationCost": 10618.6606262,
        },
        "euclidean": {
            "KRdiag": np.array([np.float64(0.002629), np.float64(0.002944), np.float64(0.1783)]),
            "Kxidiag": np.array([np.float64(32.3), np.float64(0.1116), np.float64(13.85)]),
            "LambdaDiag": np.array([np.float64(8.936), np.float64(11.53), np.float64(3.649), np.float64(0.09005), np.float64(0.1241), np.float64(0.1542)]),
            "kd": 22.1,
            "ks": 0.4749,
            "alpha": 0.947,
            "gammaE": np.array([np.float64(0.06224), np.float64(0.04911), np.float64(0.003038), np.float64(0.01321), np.float64(0.08969), np.float64(0.004635), np.float64(0.009389), np.float64(0.000774), np.float64(6e-05), np.float64(0.000156)]),
            "gammaB": 0.001,
            "optimizationCost": 79.7507,
        },
        "bregman": {
            "KRdiag": np.array([np.float64(0.002629), np.float64(0.002944), np.float64(0.1783)]),
            "Kxidiag": np.array([np.float64(32.3), np.float64(0.1116), np.float64(13.85)]),
            "LambdaDiag": np.array([np.float64(8.936), np.float64(11.53), np.float64(3.649), np.float64(0.09005), np.float64(0.1241), np.float64(0.1542)]),
            "kd": 22.1,
            "ks": 0.4749,
            "alpha": 0.947,
            "gammaE": np.array([np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001)]),
            "gammaB": 0.09054,
            "optimizationCost": 61.3517,
        },
    }
    if key in registry:
        return registry[key]
    if m in registry:
        return registry[m]
    raise ValueError(f"Unknown mode/coriolis combination: {mode}/{coriolis}")
