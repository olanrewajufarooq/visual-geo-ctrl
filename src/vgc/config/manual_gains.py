"""Manual nominal gain registry."""
import numpy as np


def manual_gains(mode: str, coriolis: str) -> dict:
    if str(mode).lower() != "nominal" or str(coriolis).lower() not in {"lc", "rb"}:
        raise KeyError(f"Unknown nominal scenario: {mode}/{coriolis}")
    gains = {"KRdiag": [0.01, 0.01, 2.0], "Kxidiag": [0.4, 0.4, 0.4],
             "LambdaDiag": [0.1, 0.1, 0.1, 0.1638, 0.2499, 0.1605],
             "kd": 1.5, "ks": 1.0, "alpha": 0.5}
    if str(coriolis).lower() == "rb":
        gains.update(KRdiag=[100.0, 100.0, 200.0], Kxidiag=[5.0, 5.0, 5.0],
                     LambdaDiag=[100.0, 100.0, 100.0, 10.0, 10.0, 10.0])
    return {key: np.array(value, dtype=float, copy=True) if isinstance(value, list) else value
            for key, value in gains.items()}
