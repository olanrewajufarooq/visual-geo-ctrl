"""Nominal gains promoted from the predecessor project's tuning run."""

import numpy as np


_REGISTRY = {
    "lc": {
        "KRdiag": np.array([0.03352, 0.0217, 0.003367]),
        "Kxidiag": np.array([31.96, 25.52, 8.649]),
        "LambdaDiag": np.array([46.79, 29.15, 21.13, 0.09285, 0.1236, 0.06351]),
        "kd": 14.14,
        "ks": 10.31,
        "alpha": 0.8453,
    },
    "rb": {
        "KRdiag": np.array([3.718, 3.355, 2.812]),
        "Kxidiag": np.array([51.77, 22.57, 12.15]),
        "LambdaDiag": np.array([1.078, 1.088, 1.424, 0.2863, 0.5781, 0.2203]),
        "kd": 5.111,
        "ks": 3.23,
        "alpha": 0.95,
    },
}


def manual_gains(mode: str, coriolis: str) -> dict:
    """Return the promoted nominal gain set for LC or RB."""
    if str(mode).lower() != "nominal":
        raise KeyError(f"Unknown nominal mode: {mode!r}")
    key = str(coriolis).lower()
    if key not in _REGISTRY:
        raise KeyError(f"Unknown Coriolis form: {coriolis!r}")
    return {
        name: np.array(value, dtype=float, copy=True) if isinstance(value, np.ndarray) else value
        for name, value in _REGISTRY[key].items()
    }
