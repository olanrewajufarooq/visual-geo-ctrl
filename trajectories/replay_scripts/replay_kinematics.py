"""Decode replay CSVs and construct body-frame kinematics."""

import csv
import os
from pathlib import Path
from typing import Dict, Any, Union
import numpy as np

from .replay_wnoj_smoother import ReplayWnojSmoother


class ReplayKinematics:
    """Decode flight CSV telemetry and construct body-frame kinematics."""

    @staticmethod
    def process_single(
        raw_path: Union[str, Path],
        replay_id: str,
        entry: Dict[str, Any],
    ) -> Dict[str, Any]:
        raw_path = Path(raw_path)
        if not raw_path.is_file():
            raise FileNotFoundError(f"Replay source file '{raw_path}' does not exist.")

        # Read CSV with standard library csv
        with open(raw_path, "r", encoding="utf-8", newline="") as f:
            reader = csv.reader(f)
            header = next(reader)
            cols = [
                "elapsed_time",
                "drone_x",
                "drone_y",
                "drone_z",
                "drone_velocity_linear_x",
                "drone_velocity_linear_y",
                "drone_velocity_linear_z",
                "drone_velocity_angular_x",
                "drone_velocity_angular_y",
                "drone_velocity_angular_z",
            ] + [f"drone_rot[{i}]" for i in range(9)]

            col_indices = [header.index(c) for c in cols]
            rows = []
            for row in reader:
                if row:
                    rows.append([float(row[i]) for i in col_indices])

        data = np.asarray(rows, dtype=float)
        t = data[:, 0]
        p = data[:, 1:4]
        v_world = data[:, 4:7]
        omega_world = data[:, 7:10]
        rot_raw = data[:, 10:19]

        n = len(t)
        R = np.zeros((3, 3, n), dtype=float)
        v_b = np.zeros((n, 3), dtype=float)
        omega_b = np.zeros((n, 3), dtype=float)

        for k in range(n):
            # Row-major 3x3 rotation matrix from body to world
            R_wb = rot_raw[k].reshape((3, 3))
            R_bw = R_wb.T
            R[:, :, k] = R_wb

            v_b[k, :] = R_bw @ v_world[k]
            omega_b[k, :] = R_bw @ omega_world[k]

        options = ReplayKinematics._postprocessing_options(entry)
        smoother = ReplayWnojSmoother(options)
        smoother.fit(t, R, p, np.hstack([omega_b, v_b]))

        R_out, p_out, omega_b_out, v_b_out, alpha_b_out, a_b_out = (
            ReplayKinematics._sample_smoother(smoother)
        )

        sample_rate = ReplayKinematics.estimate_sample_rate(t)

        meta = {
            "id": replay_id,
            "source_mode": str(entry.get("source_mode", "autonomous")),
            "source_file": str(entry.get("source_file", "")),
            "sampleRateHz": sample_rate,
            "accelerationMethod": "wnoj",
            "tStart": float(t[0]),
            "tEnd": float(t[-1]),
            "postprocessingMethod": smoother.method,
            "postprocessingDiagnostics": smoother.diagnostics,
        }

        traj = {
            "t": t.reshape((-1,)),
            "p": p_out,
            "v_b": v_b_out,
            "a_b": a_b_out,
            "omega_b": omega_b_out,
            "alpha_b": alpha_b_out,
            "R": R_out,
            "meta": meta,
        }
        return traj

    @staticmethod
    def _postprocessing_options(entry: Dict[str, Any]) -> Dict[str, Any]:
        options = {}
        if isinstance(entry, dict) and "postprocessing" in entry:
            pp = entry["postprocessing"]
            if isinstance(pp, dict) and "wnoj" in pp:
                options = pp["wnoj"]
        return options

    @staticmethod
    def _sample_smoother(smoother: ReplayWnojSmoother):
        times = smoother.output_times
        n = len(times)
        R = np.zeros((3, 3, n), dtype=float)
        p = np.zeros((n, 3), dtype=float)
        omega_b = np.zeros((n, 3), dtype=float)
        v_b = np.zeros((n, 3), dtype=float)
        alpha_b = np.zeros((n, 3), dtype=float)
        a_b = np.zeros((n, 3), dtype=float)

        for k in range(n):
            H, V, A = smoother.evaluate(times[k])
            R[:, :, k] = H[0:3, 0:3]
            p[k, :] = H[0:3, 3]
            omega_b[k, :] = V[0:3]
            v_b[k, :] = V[3:6]
            alpha_b[k, :] = A[0:3]
            a_b[k, :] = A[3:6]

        return R, p, omega_b, v_b, alpha_b, a_b

    @staticmethod
    def estimate_sample_rate(t: np.ndarray) -> float:
        dt = np.diff(t.ravel())
        if len(dt) == 0:
            return 0.0
        med = float(np.median(dt))
        return 1.0 / med if med > 0 else 0.0
