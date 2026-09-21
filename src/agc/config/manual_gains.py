"""Manual gain configurations for AGC scenarios."""

import numpy as np


def general_default():
    return {
        "KRdiag": np.array([0.01, 0.01, 2.0]),
        "Kxidiag": np.array([0.4, 0.4, 0.4]),
        "LambdaDiag": np.array([0.1, 0.1, 0.1, 0.1638, 0.2499, 0.1605]),
        "kd": 1.5,
        "ks": 1.0,
        "alpha": 0.5,
        "gammaE": 0.001 * np.ones(10),
        "gammaB": 0.001,
    }


def manual_gains(mode: str, coriolis: str) -> dict:
    """Return hand-tuned gains for given controller mode and Coriolis factorization."""
    key = f"{mode.lower()}_{coriolis.lower()}"
    gains = general_default()
    if key == "nominal_c2":
        gains["KRdiag"] = np.array([100.0, 100.0, 200.0])
        gains["Kxidiag"] = np.array([5.0, 5.0, 5.0])
        gains["LambdaDiag"] = np.array([100.0, 100.0, 100.0, 10.0, 10.0, 10.0])
    return gains
