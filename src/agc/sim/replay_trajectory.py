"""Replay trajectory loader and continuous SE(3) sampler."""

import os
from typing import Callable, Dict, Any
import numpy as np
import scipy.io as sio

from ..math.se3 import rotm_to_quat, quat_to_rotm
from .validation import validate_replay_data


class ReplayTrajectory:
    """Continuous SE(3) trajectory sampler from recorded flight data."""

    def __init__(self, file_path: str):
        if not os.path.isfile(file_path):
            raise FileNotFoundError(f"Replay artifact not found: {file_path}")

        mat = sio.loadmat(file_path, squeeze_me=True)
        if "traj" not in mat:
            raise KeyError(f"Replay artifact {file_path} must contain 'traj'.")

        raw_traj = mat["traj"]
        # Extract fields whether raw_traj is a void/struct or dict
        if hasattr(raw_traj, "dtype") and raw_traj.dtype.names:
            names = raw_traj.dtype.names
            self.traj = {name: raw_traj[name].item() if raw_traj[name].shape == () else raw_traj[name] for name in names}
        elif isinstance(raw_traj, dict):
            self.traj = raw_traj
        else:
            raise TypeError(f"Unexpected traj type: {type(raw_traj)}")

        validate_replay_data(self.traj)

        self.t = np.asarray(self.traj["t"], dtype=float).ravel()
        self.p = np.asarray(self.traj["p"], dtype=float)
        self.v_b = np.asarray(self.traj["v_b"], dtype=float)
        self.a_b = np.asarray(self.traj["a_b"], dtype=float)
        self.omega_b = np.asarray(self.traj["omega_b"], dtype=float)
        self.alpha_b = np.asarray(self.traj["alpha_b"], dtype=float)
        self.R = np.asarray(self.traj["R"], dtype=float)

    def sample(self, time: float) -> Dict[str, np.ndarray]:
        """Sample desired SE(3) state H, V, Vdot at the given time."""
        time = float(np.clip(time, self.t[0], self.t[-1]))

        # Linear interpolation for Euclidean quantities
        p = np.array([np.interp(time, self.t, self.p[:, j]) for j in range(3)], dtype=float)
        v = np.array([np.interp(time, self.t, self.v_b[:, j]) for j in range(3)], dtype=float)
        a = np.array([np.interp(time, self.t, self.a_b[:, j]) for j in range(3)], dtype=float)
        omega = np.array([np.interp(time, self.t, self.omega_b[:, j]) for j in range(3)], dtype=float)
        alpha = np.array([np.interp(time, self.t, self.alpha_b[:, j]) for j in range(3)], dtype=float)

        # SLERP for rotation matrix
        idx = np.searchsorted(self.t, time)
        if idx == 0:
            R = self.R[:, :, 0]
        elif idx >= len(self.t):
            R = self.R[:, :, -1]
        else:
            lower = idx - 1
            upper = idx
            ratio = (time - self.t[lower]) / (self.t[upper] - self.t[lower])
            q0 = rotm_to_quat(self.R[:, :, lower])
            q1 = rotm_to_quat(self.R[:, :, upper])
            if np.dot(q0, q1) < 0.0:
                q1 = -q1
            q = (1.0 - ratio) * q0 + ratio * q1
            q = q / np.linalg.norm(q)
            R = quat_to_rotm(q)

        H = np.eye(4, dtype=float)
        H[0:3, 0:3] = R
        H[0:3, 3] = p
        V = np.concatenate([omega, v])
        Vdot = np.concatenate([alpha, a])
        return {"H": H, "V": V, "Vdot": Vdot}

    def __call__(self, time: float) -> Dict[str, np.ndarray]:
        return self.sample(time)


def replay_trajectory(file_path: str) -> Callable[[float], Dict[str, np.ndarray]]:
    """Factory function matching MATLAB agc.sim.replayTrajectory."""
    sampler = ReplayTrajectory(file_path)
    return sampler.sample
