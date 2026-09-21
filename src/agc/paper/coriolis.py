"""Coriolis factorizations for SE(3) rigid-body dynamics."""

import numpy as np
from ..math.se3 import ad_twist


def coriolis(form: str, V: np.ndarray, I6: np.ndarray, U: np.ndarray) -> np.ndarray:
    """Evaluate paper C_i(V, I) U action.

    Parameters:
    -----------
    form : str
        'c1' (Levi-Civita connection) or 'c2' (coadjoint action).
    V : (6,) array-like
        Differentiation direction twist [omega; v].
    I6 : (6, 6) array-like
        Generalized inertia matrix.
    U : (6,) array-like
        Tangent vector twist acted upon.
    """
    V = np.asarray(V, dtype=float).ravel()
    U = np.asarray(U, dtype=float).ravel()
    I6 = np.asarray(I6, dtype=float)
    adV = ad_twist(V)
    adU = ad_twist(U)
    form_lower = form.lower()
    if form_lower == "c1":
        # Left-trivialized Levi-Civita factorization
        return 0.5 * (I6 @ adV @ U - adU.T @ (I6 @ V) - adV.T @ (I6 @ U))
    elif form_lower == "c2":
        # Alternative coadjoint factorization: C_2(V, I)U = -ad_U^*(I * V)
        return -adU.T @ (I6 @ V)
    else:
        raise ValueError(f"Unknown Coriolis form '{form}'. Must be 'c1' or 'c2'.")
