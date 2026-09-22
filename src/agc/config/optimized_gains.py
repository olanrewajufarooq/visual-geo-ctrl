"""Optimized per-scenario gain registry.
Last promotion: 2026-09-22 18:24:48 (Stage: dissipation, Cost: 2761.57)
"""

import numpy as np


def optimized_gains(mode: str, coriolis: str) -> dict:
    """Return tuned per-scenario gains for each controller variant."""
    key = f"{mode.lower()}_{coriolis.lower()}"
    registry = {
        "nominal_c1": {
            "KRdiag": np.array([np.float64(8.184), np.float64(12.54), np.float64(2.901)]),
            "Kxidiag": np.array([np.float64(61.01), np.float64(92.17), np.float64(67.64)]),
            "LambdaDiag": np.array([np.float64(7.373), np.float64(7.3), np.float64(4.846), np.float64(0.114), np.float64(0.1389), np.float64(0.1186)]),
            "kd": 25.58,
            "ks": 0.006322,
            "alpha": 0.5771,
            "gammaE": np.array([np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001)]),
            "gammaB": 0.001,
            "optimizationCost": 2789.12868478,
        },
        "nominal_c2": {
            "KRdiag": np.array([np.float64(3.013), np.float64(3.439), np.float64(2.863)]),
            "Kxidiag": np.array([np.float64(47.12), np.float64(23.46), np.float64(23.06)]),
            "LambdaDiag": np.array([np.float64(1.078), np.float64(1.087), np.float64(1.424), np.float64(0.2865), np.float64(0.578), np.float64(0.2202)]),
            "kd": 3.337,
            "ks": 5.243,
            "alpha": 0.9019,
            "gammaE": np.array([np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001)]),
            "gammaB": 0.001,
            "optimizationCost": 2761.57419232,
        },
        "euclidean_c1": {
            "KRdiag": np.array([np.float64(0.3257), np.float64(30.0), np.float64(0.9026)]),
            "Kxidiag": np.array([np.float64(15.86), np.float64(300.0), np.float64(17.1)]),
            "LambdaDiag": np.array([np.float64(5.18), np.float64(5.943), np.float64(0.8447), np.float64(0.06595), np.float64(0.03169), np.float64(0.05616)]),
            "kd": 0.7107,
            "ks": 1.997,
            "alpha": 0.2557,
            "gammaE": np.array([np.float64(0.9116), np.float64(0.03459), np.float64(0.04773), np.float64(0.1066), np.float64(0.01137), np.float64(5.6e-05), np.float64(0.02999), np.float64(0.02285), np.float64(0.006966), np.float64(0.002507)]),
            "gammaB": 0.001,
            "optimizationCost": 39.6447473774,
        },
        "euclidean_c2": {
            "KRdiag": np.array([np.float64(0.03053), np.float64(0.001), np.float64(0.2615)]),
            "Kxidiag": np.array([np.float64(0.6975), np.float64(0.1463), np.float64(0.2252)]),
            "LambdaDiag": np.array([np.float64(1.145), np.float64(5.131), np.float64(5.867), np.float64(0.02178), np.float64(7.857), np.float64(0.02619)]),
            "kd": 3.988,
            "ks": 4.345,
            "alpha": 0.8862,
            "gammaE": np.array([np.float64(0.000135), np.float64(0.000978), np.float64(1.4e-05), np.float64(3.5e-05), np.float64(2.2e-05), np.float64(2.8e-05), np.float64(0.001193), np.float64(0.005383), np.float64(0.000146), np.float64(0.000418)]),
            "gammaB": 0.001,
            "optimizationCost": 2731.79869078,
        },
        "bregman_c1": {
            "KRdiag": np.array([np.float64(0.1517), np.float64(16.04), np.float64(0.009427)]),
            "Kxidiag": np.array([np.float64(154.3), np.float64(28.49), np.float64(13.21)]),
            "LambdaDiag": np.array([np.float64(2.393), np.float64(35.39), np.float64(4.724), np.float64(0.04628), np.float64(0.04214), np.float64(0.2099)]),
            "kd": 4.974,
            "ks": 9.624,
            "alpha": 0.05,
            "gammaE": np.array([np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001)]),
            "gammaB": 0.1,
            "optimizationCost": 58.1586130316,
        },
        "bregman_c2": {
            "KRdiag": np.array([np.float64(0.6083), np.float64(6.704), np.float64(0.002184)]),
            "Kxidiag": np.array([np.float64(300.0), np.float64(277.5), np.float64(63.57)]),
            "LambdaDiag": np.array([np.float64(6.834), np.float64(13.47), np.float64(0.2949), np.float64(0.02961), np.float64(0.02548), np.float64(0.01474)]),
            "kd": 4.145,
            "ks": 0.3445,
            "alpha": 0.05954,
            "gammaE": np.array([np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001)]),
            "gammaB": 0.1,
            "optimizationCost": 72.4986711451,
        },
    }
    if key not in registry:
        raise ValueError(f"Unknown mode/coriolis combination: {mode}/{coriolis}")
    return registry[key]
