"""SE(3) and SO(3) Lie group and Lie algebra operations."""

import numpy as np


def skew(v: np.ndarray) -> np.ndarray:
    """Return 3x3 skew-symmetric matrix: skew(v) @ y == cross(v, y)."""
    v = np.asarray(v, dtype=float).ravel()
    return np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0]
    ], dtype=float)


def unskew(X: np.ndarray) -> np.ndarray:
    """Inverse coordinate map for a 3-by-3 skew-symmetric matrix."""
    X = np.asarray(X, dtype=float)
    return np.array([X[2, 1], X[0, 2], X[1, 0]], dtype=float)


def ad_twist(V: np.ndarray) -> np.ndarray:
    """Lie-bracket matrix on se(3) for twist V = [omega; v]."""
    V = np.asarray(V, dtype=float).ravel()
    sw_om = skew(V[0:3])
    sw_v = skew(V[3:6])
    return np.block([
        [sw_om, np.zeros((3, 3))],
        [sw_v, sw_om]
    ])


def adjoint_se3(H: np.ndarray) -> np.ndarray:
    """Adjoint matrix Ad_H for twists ordered as [angular; linear]."""
    H = np.asarray(H, dtype=float)
    R = H[0:3, 0:3]
    p = H[0:3, 3]
    return np.block([
        [R, np.zeros((3, 3))],
        [skew(p) @ R, R]
    ])


def inv_se3(H: np.ndarray) -> np.ndarray:
    """Inverse homogeneous transformation matrix in SE(3)."""
    H = np.asarray(H, dtype=float)
    R = H[0:3, 0:3]
    p = H[0:3, 3]
    H_inv = np.eye(4, dtype=float)
    H_inv[0:3, 0:3] = R.T
    H_inv[0:3, 3] = -R.T @ p
    return H_inv


def expm_so3(w: np.ndarray) -> np.ndarray:
    """Analytical matrix exponential for so(3) via Rodrigues' formula."""
    w = np.asarray(w, dtype=float).ravel()
    theta = np.linalg.norm(w)
    if theta < 1e-12:
        return np.eye(3, dtype=float) + skew(w)
    K = skew(w / theta)
    return np.eye(3, dtype=float) + np.sin(theta) * K + (1.0 - np.cos(theta)) * (K @ K)


def rotm_to_quat(R: np.ndarray) -> np.ndarray:
    """Convert 3x3 rotation matrix to quaternion [x, y, z, w] (PyBullet convention)."""
    R = np.asarray(R, dtype=float)
    tr = np.trace(R)
    if tr > 0.0:
        S = np.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S
    q = np.array([qx, qy, qz, qw], dtype=float)
    return q / np.linalg.norm(q)


def quat_to_rotm(q: np.ndarray) -> np.ndarray:
    """Convert quaternion [x, y, z, w] to 3x3 rotation matrix."""
    q = np.asarray(q, dtype=float).ravel()
    q = q / np.linalg.norm(q)
    x, y, z, w = q
    return np.array([
        [1.0 - 2.0 * (y*y + z*z), 2.0 * (x*y - z*w),       2.0 * (x*z + y*w)],
        [2.0 * (x*y + z*w),       1.0 - 2.0 * (x*x + z*z), 2.0 * (y*z - x*w)],
        [2.0 * (x*z - y*w),       2.0 * (y*z + x*w),       1.0 - 2.0 * (x*x + y*y)]
    ], dtype=float)


def log_so3(R: np.ndarray) -> np.ndarray:
    """Analytical matrix logarithm for SO(3) returning 3-element rotation vector."""
    R = np.asarray(R, dtype=float)
    tr = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    theta = np.arccos(tr)
    if theta < 1e-10:
        return unskew(R - np.eye(3))
    return unskew((theta / (2.0 * np.sin(theta))) * (R - R.T))


def exp_se3(xi: np.ndarray) -> np.ndarray:
    """Exponential map on SE(3) mapping twist xi = [omega; v] to 4x4 transform."""
    xi = np.asarray(xi, dtype=float).ravel()
    omega = xi[0:3]
    v = xi[3:6]
    theta = np.linalg.norm(omega)
    R = expm_so3(omega)
    if theta < 1e-10:
        V_mat = np.eye(3, dtype=float) + 0.5 * skew(omega)
    else:
        K = skew(omega / theta)
        V_mat = np.eye(3, dtype=float) + ((1.0 - np.cos(theta)) / theta) * K + ((theta - np.sin(theta)) / theta) * (K @ K)
    p = V_mat @ v
    H = np.eye(4, dtype=float)
    H[0:3, 0:3] = R
    H[0:3, 3] = p
    return H


def log_se3(H: np.ndarray) -> np.ndarray:
    """Logarithm map on SE(3) mapping 4x4 transform to twist xi = [omega; v]."""
    H = np.asarray(H, dtype=float)
    R = H[0:3, 0:3]
    p = H[0:3, 3]
    omega = log_so3(R)
    theta = np.linalg.norm(omega)
    if theta < 1e-10:
        V_inv = np.eye(3, dtype=float) - 0.5 * skew(omega)
    else:
        K = skew(omega / theta)
        half_theta = 0.5 * theta
        coeff = 1.0 - (half_theta / np.tan(half_theta))
        V_inv = np.eye(3, dtype=float) - 0.5 * skew(omega) + coeff * (K @ K)
    v = V_inv @ p
    return np.concatenate([omega, v])

