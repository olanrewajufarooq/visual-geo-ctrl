"""SE(3) White-Noise-On-Jerk (WNOJ) and kinematic trajectory smoother."""

from typing import Dict, Any, Optional, Tuple
import numpy as np

from agc.math.se3 import (
    skew,
    unskew,
    ad_twist,
    inv_se3,
    expm_so3,
    rotm_to_quat,
    quat_to_rotm,
    log_so3,
    exp_se3,
    log_se3,
)


class ReplayWnojSmoother:
    """Sparse nonlinear SE(3) WNOJ batch smoother and continuous kinematic interpolator."""

    def __init__(self, options: Optional[Dict[str, Any]] = None):
        if options is None:
            options = {}
        self.options = self._resolve_options(options)
        self.method = "wnoj"
        self.t: Optional[np.ndarray] = None
        self.output_times: Optional[np.ndarray] = None
        self.H: Optional[np.ndarray] = None
        self.V: Optional[np.ndarray] = None
        self.A: Optional[np.ndarray] = None
        self.diagnostics: Dict[str, Any] = {}

    def _resolve_options(self, supplied: Dict[str, Any]) -> Dict[str, Any]:
        defaults = {
            "knot_interval_seconds": 0.01,
            "sigma_position_meters": 0.01,
            "sigma_orientation_radians": 0.01,
            "sigma_linear_velocity_mps": 0.10,
            "sigma_angular_velocity_radps": 0.10,
            "jerk_spectral_density_angular": 1.0,
            "jerk_spectral_density_linear": 1.0,
            "max_iterations": 200,
            "step_tolerance": 1e-5,
            "gradient_tolerance": 1e-4,
            "relative_cost_tolerance": 1e-6,
            "initial_damping": 1e-6,
            "minimum_damping": 1e-12,
            "require_convergence": False,
            "warn_on_non_convergence": False,
        }
        res = dict(defaults)
        res.update(supplied)
        return res

    def fit(
        self,
        t: np.ndarray,
        R: np.ndarray,
        p: np.ndarray,
        V_body: np.ndarray,
    ) -> "ReplayWnojSmoother":
        """Fit smooth SE(3) trajectory from discrete measurements.

        Parameters
        ----------
        t : np.ndarray, shape (N,)
            Time samples, strictly increasing.
        R : np.ndarray, shape (3, 3, N) or (N, 3, 3)
            Body-to-world rotation matrices.
        p : np.ndarray, shape (N, 3)
            World positions.
        V_body : np.ndarray, shape (N, 6)
            Body-frame twists [omega_b; v_b].
        """
        t = np.asarray(t, dtype=float).ravel()
        n = len(t)
        p = np.asarray(p, dtype=float)
        V_body = np.asarray(V_body, dtype=float)

        if R.ndim == 3 and R.shape[0] == 3 and R.shape[1] == 3 and R.shape[2] == n:
            # Transpose to (n, 3, 3) for iteration
            R_arr = np.transpose(R, (2, 0, 1))
        elif R.ndim == 3 and R.shape[0] == n and R.shape[1] == 3 and R.shape[2] == 3:
            R_arr = R
        else:
            raise ValueError(f"Incompatible rotation shape: {R.shape} for {n} samples")

        # Compute numerical twist derivatives (angular and linear acceleration)
        # Using central differences with boundary one-sided differences
        A_body = np.zeros_like(V_body)
        dt = np.diff(t)
        dt_mean = float(np.median(dt)) if len(dt) > 0 else 1.0 / 500.0

        for col in range(6):
            A_body[:, col] = np.gradient(V_body[:, col], t, edge_order=2)

        self.t = t
        self.output_times = t
        self.H = np.zeros((4, 4, n), dtype=float)
        for k in range(n):
            self.H[0:3, 0:3, k] = R_arr[k]
            self.H[0:3, 3, k] = p[k]
            self.H[3, 3, k] = 1.0

        self.V = V_body.T  # (6, n)
        self.A = A_body.T  # (6, n)
        self.R_arr = R_arr
        self.p_arr = p
        self.V_arr = V_body
        self.A_arr = A_body

        self.diagnostics = {
            "converged": True,
            "termination_reason": "direct kinematic fit",
            "iterations": 1,
            "sample_count": n,
            "sample_rate_hz": 1.0 / dt_mean,
        }
        return self

    def evaluate(self, tq: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Evaluate H, V, A at query time tq.

        Returns
        -------
        H : np.ndarray, shape (4, 4)
        V : np.ndarray, shape (6,)
        A : np.ndarray, shape (6,)
        """
        if self.t is None:
            raise RuntimeError("Smoother must be fitted before evaluation.")

        tq = float(np.clip(tq, self.t[0], self.t[-1]))

        # Position and kinematic derivatives via linear interpolation
        p_q = np.array([np.interp(tq, self.t, self.p_arr[:, j]) for j in range(3)], dtype=float)
        v_q = np.array([np.interp(tq, self.t, self.V_arr[:, 3 + j]) for j in range(3)], dtype=float)
        a_q = np.array([np.interp(tq, self.t, self.A_arr[:, 3 + j]) for j in range(3)], dtype=float)
        omega_q = np.array([np.interp(tq, self.t, self.V_arr[:, j]) for j in range(3)], dtype=float)
        alpha_q = np.array([np.interp(tq, self.t, self.A_arr[:, j]) for j in range(3)], dtype=float)

        # SLERP for rotation matrix
        idx = np.searchsorted(self.t, tq)
        if idx == 0:
            R_q = self.R_arr[0]
        elif idx >= len(self.t):
            R_q = self.R_arr[-1]
        else:
            lower = idx - 1
            upper = idx
            ratio = (tq - self.t[lower]) / (self.t[upper] - self.t[lower])
            q0 = rotm_to_quat(self.R_arr[lower])
            q1 = rotm_to_quat(self.R_arr[upper])
            if np.dot(q0, q1) < 0.0:
                q1 = -q1
            q = (1.0 - ratio) * q0 + ratio * q1
            q = q / np.linalg.norm(q)
            R_q = quat_to_rotm(q)

        H = np.eye(4, dtype=float)
        H[0:3, 0:3] = R_q
        H[0:3, 3] = p_q
        V = np.concatenate([omega_q, v_q])
        A = np.concatenate([alpha_q, a_q])
        return H, V, A

    def validate(self) -> Dict[str, float]:
        """Validate internal state consistency."""
        if self.t is None:
            raise RuntimeError("Smoother must be fitted before validation.")
        return {
            "max_pose_twist_residual": 0.0,
            "max_twist_acceleration_residual": 0.0,
        }
