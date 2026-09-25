"""Regressor matrix assembly for SE(3) paper tracking controller."""

import numpy as np
from ..math.se3 import skew, ad_twist


def inertial_block(twist: np.ndarray) -> np.ndarray:
    """Return 6x10 Y_I([alpha; beta]) from the paper derivation.

    Parameter order: pi = [m; hx, hy, hz; Ixx, Iyy, Izz, Ixy, Ixz, Iyz].
    """
    twist = np.asarray(twist, dtype=float).ravel()
    alpha = twist[0:3]
    beta = twist[3:6]

    # I_b * alpha is linear in [Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
    B = np.array([
        [alpha[0], 0.0,      0.0,      alpha[1], alpha[2], 0.0],
        [0.0,      alpha[1], 0.0,      alpha[0], 0.0,      alpha[2]],
        [0.0,      0.0,      alpha[2], 0.0,      alpha[0], alpha[1]]
    ], dtype=float)

    return np.block([
        [np.zeros((3, 1), dtype=float), -skew(beta), B],
        [beta.reshape((3, 1)),           skew(alpha), np.zeros((3, 6), dtype=float)]
    ])


def coriolis_block(form: str, V: np.ndarray, Vr: np.ndarray) -> np.ndarray:
    """Return the explicit 6x10 C_K(V, I) * Vr coefficient."""
    adV = ad_twist(V)
    adVr = ad_twist(Vr)
    form_lower = str(form).strip().lower()

    if form_lower == "lc":
        # C_LC is the Levi-Civita factorization
        return 0.5 * (
            inertial_block(adV @ Vr)
            - adVr.T @ inertial_block(V)
            - adV.T @ inertial_block(Vr)
        )
    elif form_lower == "rb":
        # C_RB(V, I)Vr = -ad_Vr^* (I * V)
        return -adVr.T @ inertial_block(V)
    else:
        raise ValueError(f"Unknown form '{form}'. Must be 'lc' ($C_{{\\mathrm{{LC}}}}$) or 'rb' ($C_{{\\mathrm{{RB}}}}$).")


def gravity_block(H: np.ndarray, gravity: np.ndarray) -> np.ndarray:
    """Return 6x10 Y_g for the paper's body-frame gravity convention."""
    H = np.asarray(H, dtype=float)
    gravity = np.asarray(gravity, dtype=float).ravel()
    g_body = H[0:3, 0:3].T @ gravity

    return np.block([
        [np.zeros((3, 1), dtype=float), -skew(g_body),               np.zeros((3, 6), dtype=float)],
        [g_body.reshape((3, 1)),         np.zeros((3, 3), dtype=float), np.zeros((3, 6), dtype=float)]
    ])


def regressor(
    H: np.ndarray,
    V: np.ndarray,
    Vr: np.ndarray,
    VrDot: np.ndarray,
    gravity: np.ndarray,
    form: str,
) -> np.ndarray:
    """Assemble 6-by-10 reference-model wrench regressor Y satisfying:
    Y @ pi == I @ VrDot + C_K(V, I) @ Vr + W_g(H, I).
    """
    YI = inertial_block(VrDot)
    Yc = coriolis_block(form, V, Vr)
    Yg = gravity_block(H, gravity)
    return YI + Yc + Yg
