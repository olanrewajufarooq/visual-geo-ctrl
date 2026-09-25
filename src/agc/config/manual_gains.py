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
    baseline = general_default()
    registry = {
        "nominal_lc": baseline,
        "nominal_rb": {
            **baseline,
            "KRdiag": np.array([100.0, 100.0, 200.0]),
            "Kxidiag": np.array([5.0, 5.0, 5.0]),
            "LambdaDiag": np.array([100.0, 100.0, 100.0, 10.0, 10.0, 10.0]),
        },
        "euclidean_lc": baseline,
        "euclidean_rb": baseline,
        "bregman_lc": baseline,
        "bregman_rb": baseline,
    }

    if key not in registry:
        raise KeyError(f"Unknown scenario key: '{key}'.")

    gains = registry[key]
    return {
        field: np.array(value, dtype=float, copy=True) if isinstance(value, np.ndarray) else value
        for field, value in gains.items()
    }
