"""Composite rigid-body inertial parameters for vehicle and payload."""

import numpy as np


def compound_pi(bare_pi: np.ndarray, payload: dict) -> np.ndarray:
    """Add an aligned cuboid payload to body-origin inertial parameters.

    Parameters:
    -----------
    bare_pi : (10,) array-like
        Inertial parameters [m; hx, hy, hz; Ixx, Iyy, Izz, Ixy, Ixz, Iyz].
    payload : dict
        'mass': float, 'dimensions': (3,) array-like, 'center': (3,) array-like.

    Returns:
    --------
    loaded_pi : (10,) ndarray
        Composite inertial parameters about the same body origin.
    """
    bare_pi = np.asarray(bare_pi, dtype=float).ravel()
    m_p = float(payload['mass'])
    r = np.asarray(payload['center'], dtype=float).ravel()
    d = np.asarray(payload['dimensions'], dtype=float).ravel()

    # Principal inertia of aligned cuboid about its CoM
    payload_inertia_com = (m_p / 12.0) * np.diag([
        d[1]**2 + d[2]**2,
        d[0]**2 + d[2]**2,
        d[0]**2 + d[1]**2,
    ])

    # Parallel-axis theorem to vehicle body origin
    payload_inertia_body = payload_inertia_com + m_p * (
        (r @ r) * np.eye(3, dtype=float) - np.outer(r, r)
    )

    bare_inertia = np.array([
        [bare_pi[4], bare_pi[7], bare_pi[8]],
        [bare_pi[7], bare_pi[5], bare_pi[9]],
        [bare_pi[8], bare_pi[9], bare_pi[6]],
    ], dtype=float)

    loaded_inertia = bare_inertia + payload_inertia_body

    return np.array([
        bare_pi[0] + m_p,
        bare_pi[1] + m_p * r[0],
        bare_pi[2] + m_p * r[1],
        bare_pi[3] + m_p * r[2],
        loaded_inertia[0, 0],
        loaded_inertia[1, 1],
        loaded_inertia[2, 2],
        loaded_inertia[0, 1],
        loaded_inertia[0, 2],
        loaded_inertia[1, 2],
    ], dtype=float)
