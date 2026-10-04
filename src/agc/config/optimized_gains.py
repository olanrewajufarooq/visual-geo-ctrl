"""Optimized per-scenario gain registry.
Last promotion: 2026-10-03 17:59:13 (Stage: verification, Cost: 371.723)
"""

import numpy as np


def optimized_gains(mode: str, coriolis: str) -> dict:
    """Return tuned per-scenario gains for each controller variant."""
    key = f"{mode.lower()}_{coriolis.lower()}"
    registry = {
        "nominal_lc": {
            "KRdiag": np.array([np.float64(0.03352), np.float64(0.0217), np.float64(0.003367)]),
            "Kxidiag": np.array([np.float64(31.96), np.float64(25.52), np.float64(8.649)]),
            "LambdaDiag": np.array([np.float64(46.79), np.float64(29.15), np.float64(21.13), np.float64(0.09285), np.float64(0.1236), np.float64(0.06351)]),
            "kd": 14.14,
            "ks": 10.31,
            "alpha": 0.8453,
            "gammaE": np.array([np.float64(0.001)] * 10),
            "gammaB": 0.001,
            "optimizationCost": 189.824830929,
        },
        "nominal_rb": {
            "KRdiag": np.array([np.float64(3.718), np.float64(3.355), np.float64(2.812)]),
            "Kxidiag": np.array([np.float64(51.77), np.float64(22.57), np.float64(12.15)]),
            "LambdaDiag": np.array([np.float64(1.078), np.float64(1.088), np.float64(1.424), np.float64(0.2863), np.float64(0.5781), np.float64(0.2203)]),
            "kd": 5.111,
            "ks": 3.23,
            "alpha": 0.95,
            "gammaE": np.array([np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001)]),
            "gammaB": 0.001,
            "optimizationCost": 2836.781023,
        },
        "euclidean_lc": {
            "KRdiag": np.array([np.float64(2.507), np.float64(0.004643), np.float64(2.368)]),
            "Kxidiag": np.array([np.float64(2.843), np.float64(4.466), np.float64(6.498)]),
            "LambdaDiag": np.array([np.float64(1.032), np.float64(4.493), np.float64(1.707), np.float64(0.02924), np.float64(5.154), np.float64(0.0798)]),
            "kd": 0.9206,
            "ks": 1.728,
            "alpha": 0.5898,
            "gammaE": np.array([np.float64(0.06224), np.float64(0.04911), np.float64(0.003038), np.float64(0.01321), np.float64(0.08969), np.float64(0.004635), np.float64(0.009389), np.float64(0.000774), np.float64(6e-05), np.float64(0.000156)]),
            "gammaB": 0.001,
            "optimizationCost": 79.750694309,
        },
        "euclidean_rb": {
            "KRdiag": np.array([np.float64(0.005718), np.float64(0.001965), np.float64(0.2317)]),
            "Kxidiag": np.array([np.float64(27.91), np.float64(72.81), np.float64(8.581)]),
            "LambdaDiag": np.array([np.float64(1.148), np.float64(1.368), np.float64(1.896), np.float64(0.2372), np.float64(0.261), np.float64(0.2487)]),
            "kd": 0.9262,
            "ks": 8.475,
            "alpha": 0.9486,
            "gammaE": np.array([np.float64(0.9417), np.float64(0.08606), np.float64(0.02651), np.float64(0.05273), np.float64(0.000281), np.float64(0.000113), np.float64(0.3724), np.float64(0.001736), np.float64(0.01859), np.float64(0.005186)]),
            "gammaB": 0.001,
            "optimizationCost": 80.1609405498,
        },
        "bregman_lc": {
            "KRdiag": np.array([np.float64(0.1784), np.float64(6.418), np.float64(0.4457)]),
            "Kxidiag": np.array([np.float64(9.833), np.float64(1.762), np.float64(33.96)]),
            "LambdaDiag": np.array([np.float64(4.392), np.float64(19.97), np.float64(15.26), np.float64(0.7227), np.float64(0.967), np.float64(0.1144)]),
            "kd": 7.934,
            "ks": 0.4086,
            "alpha": 0.5731,
            "gammaE": np.array([np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001)]),
            "gammaB": 0.09054,
            "optimizationCost": 61.3517065888,
        },
        "bregman_rb": {
            "KRdiag": np.array([np.float64(0.006092), np.float64(0.1423), np.float64(0.01086)]),
            "Kxidiag": np.array([np.float64(7.802), np.float64(8.541), np.float64(1.926)]),
            "LambdaDiag": np.array([np.float64(1.284), np.float64(1.088), np.float64(0.8828), np.float64(0.3279), np.float64(0.3048), np.float64(0.04811)]),
            "kd": 7.842,
            "ks": 1.109,
            "alpha": 0.5363,
            "gammaE": np.array([np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001), np.float64(0.001)]),
            "gammaB": 0.09904,
            "optimizationCost": 284.213663333,
        },
    }
    if key not in registry:
        raise ValueError(f"Unknown mode/coriolis combination: {mode}/{coriolis}")
    return registry[key]
