"""Discrete-time estimators for Euclidean and Bregman parameter adaptation."""

import numpy as np
from scipy.linalg import expm
from ..math.inertia import is_spd


def euclidean_step(
    pi_hat: np.ndarray, gradient: np.ndarray, Gamma: np.ndarray, dt: float
) -> np.ndarray:
    """Apply the paper's unconstrained inertial-parameter update:
    pi_hat_next = pi_hat - dt * Gamma @ gradient.
    """
    pi_hat = np.asarray(pi_hat, dtype=float).ravel()
    gradient = np.asarray(gradient, dtype=float).ravel()
    Gamma = np.asarray(Gamma, dtype=float)
    if Gamma.ndim == 1:
        Gamma = np.diag(Gamma)
    return pi_hat - dt * (Gamma @ gradient)


def bregman_step(J: np.ndarray, G: np.ndarray, gamma: float, dt: float) -> np.ndarray:
    """SPD-preserving affine-invariant pseudo-inertia update:
    J_next = J^(1/2) @ expm(-gamma * dt * J^(1/2) @ G_sym @ J^(1/2)) @ J^(1/2).
    """
    J = np.asarray(J, dtype=float)
    G = np.asarray(G, dtype=float)
    if not is_spd(J):
        raise ValueError("J must be symmetric positive definite.")

    # Eigendecomposition of symmetric J
    try:
        w, Q = np.linalg.eigh(0.5 * (J + J.T))
    except np.linalg.LinAlgError:
        return 0.5 * (J + J.T)
    w_safe = np.maximum(w, 1e-14)
    J_half = Q @ np.diag(np.sqrt(w_safe)) @ Q.T
    G_sym = 0.5 * (G + G.T)
    if not np.all(np.isfinite(G_sym)):
        return 0.5 * (J + J.T)

    # Symmetric matrix M = -gamma * dt * (J_half @ G_sym @ J_half)
    M = -gamma * dt * (J_half @ G_sym @ J_half)
    M_sym = 0.5 * (M + M.T)
    if not np.all(np.isfinite(M_sym)):
        return 0.5 * (J + J.T)
    # Spectral decomposition of symmetric matrix exponential
    try:
        w_m, Q_m = np.linalg.eigh(M_sym)
    except np.linalg.LinAlgError:
        return 0.5 * (J + J.T)
    # Bound eigenvalues in Lie algebra to prevent float64 exponential overflow
    w_m_safe = np.clip(w_m, -50.0, 50.0)
    # B = J_half @ Q_m @ diag(exp(0.5 * w_m_safe))
    B = J_half @ Q_m @ np.diag(np.exp(0.5 * w_m_safe))
    J_raw = B @ B.T
    J_raw_sym = 0.5 * (J_raw + J_raw.T)
    if not np.all(np.isfinite(J_raw_sym)):
        return 0.5 * (J + J.T)
    # Clean floating-point roundoff to guarantee SPD
    try:
        w_j, Q_j = np.linalg.eigh(J_raw_sym)
    except np.linalg.LinAlgError:
        return 0.5 * (J + J.T)
    w_j = np.maximum(w_j, 1e-14 * np.max(w_j))
    J_next = Q_j @ np.diag(w_j) @ Q_j.T
    return 0.5 * (J_next + J_next.T)


def pseudo_gradient(gpi: np.ndarray) -> np.ndarray:
    """Pull a pi-space gradient back to symmetric 4x4 pseudo-inertia matrix G."""
    gpi = np.asarray(gpi, dtype=float).ravel()
    gi = gpi[4:10]  # [Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
    G = np.zeros((4, 4), dtype=float)
    G[3, 3] = gpi[0]
    G[0:3, 3] = 0.5 * gpi[1:4]
    G[3, 0:3] = G[0:3, 3]

    G[0, 0] = gi[1] + gi[2]  # Iyy + Izz
    G[1, 1] = gi[0] + gi[2]  # Ixx + Izz
    G[2, 2] = gi[0] + gi[1]  # Ixx + Iyy

    G[0, 1] = -0.5 * gi[3]
    G[1, 0] = G[0, 1]

    G[0, 2] = -0.5 * gi[4]
    G[2, 0] = G[0, 2]

    G[1, 2] = -0.5 * gi[5]
    G[2, 1] = G[1, 2]
    return G
