"""Low-overhead telemetry objects used by live visualization consumers."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from threading import Lock
from typing import Optional

import numpy as np

from ..math.se3 import log_so3


def _copy_array(value: np.ndarray, shape: tuple[int, ...]) -> np.ndarray:
    array = np.asarray(value, dtype=float)
    if array.shape != shape:
        raise ValueError(f"Expected shape {shape}, got {array.shape}.")
    return np.array(array, dtype=float, copy=True)


@dataclass(frozen=True)
class VisualizationSnapshot:
    """Immutable, self-contained state for one live visualization sample."""

    t: float
    actual_H: np.ndarray
    desired_H: np.ndarray
    actual_V: np.ndarray
    desired_V: np.ndarray
    position_error: np.ndarray
    attitude_error: np.ndarray
    velocity_error: np.ndarray
    sliding: np.ndarray
    wrench: np.ndarray
    estimate_pi: np.ndarray
    true_pi: np.ndarray
    position_error_norm: float
    attitude_error_norm: float
    velocity_error_norm: float
    sliding_norm: float
    event: Optional[str] = None


def make_snapshot(
    *,
    t: float,
    actual_H: np.ndarray,
    desired_H: np.ndarray,
    actual_V: np.ndarray,
    desired_V: np.ndarray,
    sliding: np.ndarray,
    wrench: np.ndarray,
    estimate_pi: np.ndarray,
    true_pi: np.ndarray,
    event: Optional[str] = None,
) -> VisualizationSnapshot:
    """Build a copied snapshot and derive controller-facing error signals."""

    actual_H = _copy_array(actual_H, (4, 4))
    desired_H = _copy_array(desired_H, (4, 4))
    actual_V = _copy_array(actual_V, (6,))
    desired_V = _copy_array(desired_V, (6,))
    sliding = _copy_array(sliding, (6,))
    wrench = _copy_array(wrench, (6,))
    estimate_pi = _copy_array(estimate_pi, (10,))
    true_pi = _copy_array(true_pi, (10,))

    position_error = actual_H[:3, 3] - desired_H[:3, 3]
    attitude_error = log_so3(desired_H[:3, :3].T @ actual_H[:3, :3])
    velocity_error = actual_V - desired_V

    return VisualizationSnapshot(
        t=float(t),
        actual_H=actual_H,
        desired_H=desired_H,
        actual_V=actual_V,
        desired_V=desired_V,
        position_error=position_error.copy(),
        attitude_error=attitude_error.copy(),
        velocity_error=velocity_error.copy(),
        sliding=sliding,
        wrench=wrench,
        estimate_pi=estimate_pi,
        true_pi=true_pi,
        position_error_norm=float(np.linalg.norm(position_error)),
        attitude_error_norm=float(np.linalg.norm(attitude_error)),
        velocity_error_norm=float(np.linalg.norm(velocity_error)),
        sliding_norm=float(np.linalg.norm(sliding)),
        event=event,
    )


class LiveTelemetryBuffer:
    """Thread-safe bounded buffer that coalesces stale UI frames."""

    def __init__(self, maxsize: int = 2):
        if maxsize < 1:
            raise ValueError("maxsize must be at least one")
        self._items: deque[VisualizationSnapshot] = deque(maxlen=int(maxsize))
        self._lock = Lock()
        self.dropped_count = 0

    def publish(self, snapshot: VisualizationSnapshot) -> bool:
        with self._lock:
            if len(self._items) == self._items.maxlen:
                self._items.popleft()
                self.dropped_count += 1
            self._items.append(snapshot)
        return True

    def get_latest(self) -> Optional[VisualizationSnapshot]:
        with self._lock:
            if not self._items:
                return None
            latest = self._items[-1]
            self._items.clear()
            return latest
