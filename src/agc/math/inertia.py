"""Generalized 6D inertia and 4D pseudo-inertia mappings."""

import numpy as np
from .se3 import skew


def inertia_from_pi(pi: np.ndarray) -> np.ndarray:
    """Construct 6x6 generalized inertia matrix from 10-parameter vector:
    pi = [m; hx, hy, hz; Ixx, Iyy, Izz, Ixy, Ixz, Iyz].
    """
    pi = np.asarray(pi, dtype=float).ravel()
    m = pi[0]
    h = pi[1:4]
    Ixx, Iyy, Izz, Ixy, Ixz, Iyz = pi[4:10]
    Ib = np.array([
        [Ixx, Ixy, Ixz],
        [Ixy, Iyy, Iyz],
        [Ixz, Iyz, Izz]
    ], dtype=float)
    sh = skew(h)
    return np.block([
        [Ib, sh],
        [-sh, m * np.eye(3, dtype=float)]
    ])


def pseudo_from_pi(pi: np.ndarray) -> np.ndarray:
    """Convert 10-parameter vector pi to 4x4 symmetric pseudo-inertia J."""
    pi = np.asarray(pi, dtype=float).ravel()
    I6 = inertia_from_pi(pi)
    Ib = I6[0:3, 0:3]
    S = 0.5 * np.trace(Ib) * np.eye(3, dtype=float) - Ib
    h = pi[1:4]
    m = pi[0]
    J = np.zeros((4, 4), dtype=float)
    J[0:3, 0:3] = S
    J[0:3, 3] = h
    J[3, 0:3] = h
    J[3, 3] = m
    return 0.5 * (J + J.T)


def pi_from_pseudo(J: np.ndarray) -> np.ndarray:
    """Convert 4x4 symmetric pseudo-inertia J to 10-parameter vector pi."""
    J = np.asarray(J, dtype=float)
    J = 0.5 * (J + J.T)
    S = J[0:3, 0:3]
    Ib = np.trace(S) * np.eye(3, dtype=float) - S
    return np.array([
        J[3, 3],
        J[0, 3], J[1, 3], J[2, 3],
        Ib[0, 0], Ib[1, 1], Ib[2, 2],
        Ib[0, 1], Ib[0, 2], Ib[1, 2]
    ], dtype=float)


def is_spd(X: np.ndarray, tol: float = 1e-10) -> bool:
    """True for finite symmetric positive-definite matrices."""
    X = np.asarray(X, dtype=float)
    if X.ndim != 2 or X.shape[0] != X.shape[1] or not np.all(np.isfinite(X)):
        return False
    fro_norm = np.linalg.norm(X, 'fro')
    if np.linalg.norm(X - X.T, 'fro') > tol * max(1.0, fro_norm):
        return False
    try:
        np.linalg.cholesky(0.5 * (X + X.T))
        return True
    except np.linalg.LinAlgError:
        return False
