"""Optimized per-scenario gain registry.
Last promotion: reset – costs set to 1e6 to force full re-optimization.
"""

import numpy as np


def optimized_gains(mode: str, coriolis: str) -> dict:
    """Return tuned per-scenario gains for each controller variant."""
    key = f"{mode.lower()}_{coriolis.lower()}"
    registry = {
        "nominal_c1": {
            "KRdiag": np.array([8.184, 12.54, 2.901]),
            "Kxidiag": np.array([61.01, 92.17, 67.64]),
            "LambdaDiag": np.array([7.373, 7.3, 4.846, 0.114, 0.1389, 0.1186]),
            "kd": 25.58,
            "ks": 0.006322,
            "alpha": 0.5771,
            "gammaE": np.array([0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001]),
            "gammaB": 0.001,
            "optimizationCost": 1e6,
        },
        "nominal_c2": {
            "KRdiag": np.array([3.013, 3.439, 2.863]),
            "Kxidiag": np.array([47.12, 23.46, 23.06]),
            "LambdaDiag": np.array([1.078, 1.087, 1.424, 0.2865, 0.578, 0.2202]),
            "kd": 3.337,
            "ks": 5.243,
            "alpha": 0.9019,
            "gammaE": np.array([0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001]),
            "gammaB": 0.001,
            "optimizationCost": 1e6,
        },
        "euclidean_c1": {
            "KRdiag": np.array([0.5158, 0.1334, 6.738]),
            "Kxidiag": np.array([60.71, 14.16, 0.1977]),
            "LambdaDiag": np.array([2.025, 2.969, 0.1803, 0.01243, 1.165, 0.01912]),
            "kd": 0.7416,
            "ks": 0.3496,
            "alpha": 0.2591,
            "gammaE": np.array([0.172, 0.000433, 0.0022, 0.000448, 0.01039, 0.001544, 0.02713, 0.000506, 0.008223, 0.000286]),
            "gammaB": 0.001,
            "optimizationCost": 1e6,
        },
        "euclidean_c2": {
            "KRdiag": np.array([0.03053, 0.001, 0.2615]),
            "Kxidiag": np.array([0.6975, 0.1463, 0.2252]),
            "LambdaDiag": np.array([1.145, 5.131, 5.867, 0.02178, 7.857, 0.02619]),
            "kd": 3.988,
            "ks": 4.345,
            "alpha": 0.8862,
            "gammaE": np.array([0.000135, 0.000978, 1.4e-05, 3.5e-05, 2.2e-05, 2.8e-05, 0.001193, 0.005383, 0.000146, 0.000418]),
            "gammaB": 0.001,
            "optimizationCost": 1e6,
        },
        "bregman_c1": {
            "KRdiag": np.array([0.1517, 16.04, 0.009427]),
            "Kxidiag": np.array([154.3, 28.49, 13.21]),
            "LambdaDiag": np.array([2.393, 35.39, 4.724, 0.04628, 0.04214, 0.2099]),
            "kd": 4.974,
            "ks": 9.624,
            "alpha": 0.05,
            "gammaE": np.array([0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001]),
            "gammaB": 0.1,
            "optimizationCost": 1e6,
        },
        "bregman_c2": {
            "KRdiag": np.array([0.6083, 6.704, 0.002184]),
            "Kxidiag": np.array([300.0, 277.5, 63.57]),
            "LambdaDiag": np.array([6.834, 13.47, 0.2949, 0.02961, 0.02548, 0.01474]),
            "kd": 4.145,
            "ks": 0.3445,
            "alpha": 0.05954,
            "gammaE": np.array([0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001]),
            "gammaB": 0.1,
            "optimizationCost": 1e6,
        },
    }
    if key not in registry:
        raise ValueError(f"Unknown mode/coriolis combination: {mode}/{coriolis}")
    return registry[key]
