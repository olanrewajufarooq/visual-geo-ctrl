"""Coriolis factorizations for SE(3) rigid-body dynamics.

The controller and inertial regressor currently wire only the canonical
``lc`` and ``rb`` realizations. The three-form helpers below construct
admissible gyroscopic perturbations for analysis, but are not selectable
controller/regressor factorizations.
"""

from typing import Dict, Tuple
import numpy as np
from ..math.se3 import ad_twist


def coriolis(form: str, V: np.ndarray, I6: np.ndarray, U: np.ndarray) -> np.ndarray:
    """Evaluate paper C_K(V, I) U action for canonical connections.

    Parameters:
    -----------
    form : str
        'lc' (Levi-Civita connection C_LC) or 'rb' (rigid-body coadjoint action C_RB).
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
    form_lower = str(form).strip().lower()
    if form_lower == "lc":
        # Left-trivialized Levi-Civita factorization C_LC
        return 0.5 * (I6 @ adV @ U - adU.T @ (I6 @ V) - adV.T @ (I6 @ U))
    elif form_lower == "rb":
        # Rigid-body coadjoint factorization: C_RB(V, I)U = -ad_U^*(I * V)
        return -adU.T @ (I6 @ V)
    else:
        raise ValueError(f"Unknown Coriolis form '{form}'. Must be 'lc' ($C_{{\\mathrm{{LC}}}}$) or 'rb' ($C_{{\\mathrm{{RB}}}}$).")


def three_form_basis_k(i: int, j: int, k: int, V: np.ndarray) -> np.ndarray:
    """Evaluate elementary 6x6 K^{ijk}(V) matrix from paper Appendix C (1 <= i < j < k <= 6).

    Parameters:
    -----------
    i, j, k : int
        Basis index triple satisfying 0 <= i < j < k < 6 (0-indexed).
    V : (6,) array-like
        Velocity twist argument.

    Returns:
    --------
    K : (6, 6) ndarray
        Velocity-linear skew-symmetric matrix satisfying K(V)^T = -K(V) and K(V)V = 0.
    """
    if not (0 <= i < j < k < 6):
        raise ValueError(f"Indices must satisfy 0 <= i < j < k < 6 (got i={i}, j={j}, k={k}).")
    V = np.asarray(V, dtype=float).ravel()
    K = np.zeros((6, 6), dtype=float)
    # K^{ijk}(V) = V_i (E_{kj} - E_{jk}) + V_j (E_{ik} - E_{ki}) + V_k (E_{ji} - E_{ij})
    K[k, j] += V[i]
    K[j, k] -= V[i]
    K[i, k] += V[j]
    K[k, i] -= V[j]
    K[j, i] += V[k]
    K[i, j] -= V[k]
    return K


def coriolis_three_form(
    kappa_coeffs: Dict[Tuple[int, int, int], float],
    V: np.ndarray,
    U: np.ndarray,
) -> np.ndarray:
    """Evaluate general gyroscopic perturbation K(V)U from 20-element 3-form coordinates (Appendix C)."""
    V = np.asarray(V, dtype=float).ravel()
    U = np.asarray(U, dtype=float).ravel()
    K_total = np.zeros((6, 6), dtype=float)
    for (i, j, k), val in kappa_coeffs.items():
        if val != 0.0:
            K_total += float(val) * three_form_basis_k(i, j, k, V)
    return K_total @ U
