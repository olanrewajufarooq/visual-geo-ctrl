"""Tracking error covectors and potential functions on SE(3)."""

from typing import Tuple
import numpy as np
from ..math.se3 import skew, unskew


def potential(He: np.ndarray, KR: np.ndarray, Kxi: np.ndarray) -> Tuple[np.ndarray, float]:
    """Compute error covector e_H and configuration potential Psi.

    Parameters:
    -----------
    He : (4, 4) array-like
        Error configuration matrix H_d^{-1} * H.
    KR : (3, 3) array-like
        Attitude gain matrix.
    Kxi : (3, 3) array-like
        Position gain matrix.

    Returns:
    --------
    eH : (6,) ndarray
        Error covector [eR; ep].
    psi : float
        Scalar configuration potential.
    """
    He = np.asarray(He, dtype=float)
    KR = np.asarray(KR, dtype=float)
    Kxi = np.asarray(Kxi, dtype=float)
    R = He[0:3, 0:3]
    p = He[0:3, 3]

    eR = unskew(0.5 * (KR @ R - R.T @ KR))
    ep = R.T @ Kxi @ p
    eH = np.concatenate([eR, ep])
    psi = float(0.5 * np.trace(KR @ (np.eye(3, dtype=float) - R)) + 0.5 * (p.T @ Kxi @ p))
    return eH, psi


def potential_derivative(
    He: np.ndarray, Ve: np.ndarray, KR: np.ndarray, Kxi: np.ndarray
) -> np.ndarray:
    """Differentiate e_H along left-trivialized error motion Ve = [omega_e; v_e].

    Parameters:
    -----------
    He : (4, 4) array-like
        Error configuration matrix H_d^{-1} * H.
    Ve : (6,) array-like
        Error twist in body coordinates.
    KR : (3, 3) array-like
        Attitude gain matrix.
    Kxi : (3, 3) array-like
        Position gain matrix.

    Returns:
    --------
    eHdot : (6,) ndarray
        Time derivative of error covector [eRdot; epdot].
    """
    He = np.asarray(He, dtype=float)
    Ve = np.asarray(Ve, dtype=float).ravel()
    KR = np.asarray(KR, dtype=float)
    Kxi = np.asarray(Kxi, dtype=float)

    R = He[0:3, 0:3]
    p = He[0:3, 3]
    omega = Ve[0:3]
    v = Ve[3:6]

    Rdot = R @ skew(omega)
    eRdot = unskew(0.5 * (KR @ Rdot - Rdot.T @ KR))
    ep = R.T @ Kxi @ p
    epdot = -skew(omega) @ ep + R.T @ Kxi @ R @ v
    return np.concatenate([eRdot, epdot])
