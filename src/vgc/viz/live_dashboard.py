"""Separate-process Matplotlib dashboard for live simulation diagnostics."""

from __future__ import annotations

import multiprocessing as mp
import queue
import time
from typing import Optional

from .live_telemetry import VisualizationSnapshot
from .plot_style import color_for_series, limit_for, line_style_for_series, wrap_degrees


def _run_dashboard(
    snapshot_queue: mp.Queue,
    stop_event: mp.Event,
    title: str,
) -> None:
    """Run the Matplotlib event loop in a child process."""
    import matplotlib.pyplot as plt
    import numpy as np

    plt.ion()
    fig, axes = plt.subplots(6, 2, figsize=(14, 15), sharex=True, num=title)
    fig.suptitle(title)
    fig.subplots_adjust(hspace=0.42, wspace=0.28)
    axis_groups = {
        "position": axes[0, 0],
        "attitude": axes[0, 1],
        "linear_velocity": axes[1, 0],
        "angular_velocity": axes[1, 1],
        "force": axes[2, 0],
        "torque": axes[2, 1],
        "sliding_norm": axes[3, 0],
        "mass": axes[3, 1],
        "cog": axes[4, 0],
        "inertia_principal": axes[4, 1],
        "inertia_off_diagonal": axes[5, 0],
    }
    axes[5, 1].set_visible(False)
    labels = {
        "position": ("Position (m)", ("x", "y", "z")),
        "attitude": ("Geometric attitude error (deg)", ("rx", "ry", "rz")),
        "linear_velocity": ("Linear velocity (m/s)", ("vx", "vy", "vz")),
        "angular_velocity": ("Angular velocity (rad/s)", ("wx", "wy", "wz")),
        "force": ("Commanded force (N)", ("Fx", "Fy", "Fz")),
        "torque": ("Commanded torque (Nm)", ("Tx", "Ty", "Tz")),
        "sliding_norm": ("Sliding residual norm", ("||s||",)),
        "mass": ("Mass estimate (kg)", ("mass",)),
        "cog": ("Center of mass (m)", ("cx", "cy", "cz")),
        "inertia_principal": ("Principal inertia (kg m²)", ("Ixx", "Iyy", "Izz")),
        "inertia_off_diagonal": ("Off-diagonal inertia (kg m²)", ("Ixy", "Ixz", "Iyz")),
    }
    for key, axis in axis_groups.items():
        axis.set_title(labels[key][0])
        axis.grid(True, alpha=0.25)
        axis.legend(loc="upper left", fontsize=7, ncol=2)

    history: list[VisualizationSnapshot] = []
    last_draw_wall = 0.0
    pending: Optional[VisualizationSnapshot] = None

    while not stop_event.is_set():
        latest: Optional[VisualizationSnapshot] = None
        while True:
            try:
                latest = snapshot_queue.get_nowait()
            except queue.Empty:
                break
        if latest is not None:
            pending = latest
        now = time.perf_counter()
        if pending is not None and now - last_draw_wall >= 0.1:
            history.append(pending)
            pending = None
            _redraw_dashboard(axis_groups, history, labels, np)
            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            last_draw_wall = now
        # Keep the native window responsive without forcing a simulation tick.
        plt.pause(0.01)
        if not plt.fignum_exists(fig.number):
            stop_event.set()

    plt.close(fig)


def _redraw_dashboard(axis_groups, history, labels, np) -> None:
    """Replace line data in one bounded-cost redraw."""
    times = np.asarray([item.t for item in history], dtype=float)
    actual_position = np.asarray([item.actual_H[:3, 3] for item in history])
    desired_position = np.asarray([item.desired_H[:3, 3] for item in history])
    attitude = wrap_degrees(np.degrees(np.asarray([item.attitude_error for item in history])))
    actual_v = np.asarray([item.actual_V for item in history])
    desired_v = np.asarray([item.desired_V for item in history])
    wrench = np.asarray([item.wrench for item in history])
    sliding = np.asarray([item.sliding for item in history])
    estimate = np.asarray([item.estimate_pi for item in history])
    truth = np.asarray([item.true_pi for item in history])
    estimate_com = _center_of_mass(estimate, np)
    truth_com = _center_of_mass(truth, np)

    series = {
        "position": (actual_position, desired_position, ("actual", "reference")),
        "attitude": (attitude, None, ("error",)),
        "linear_velocity": (actual_v[:, 3:6], desired_v[:, 3:6], ("actual", "reference")),
        "angular_velocity": (actual_v[:, 0:3], desired_v[:, 0:3], ("actual", "reference")),
        "force": (wrench[:, 3:6], None, ("command",)),
        "torque": (wrench[:, 0:3], None, ("command",)),
        "sliding_norm": (np.linalg.norm(sliding, axis=1, keepdims=True), None, ("",)),
        "mass": (estimate[:, 0:1], truth[:, 0:1], ("estimate", "truth")),
        "cog": (estimate_com, truth_com, ("estimate", "truth")),
        "inertia_principal": (estimate[:, 4:7], truth[:, 4:7], ("estimate", "truth")),
        "inertia_off_diagonal": (estimate[:, 7:10], truth[:, 7:10], ("estimate", "truth")),
    }
    families = {
        "position": "position",
        "attitude": "orientation",
        "linear_velocity": "linear_velocity",
        "angular_velocity": "angular_velocity",
        "force": "force",
        "torque": "torque",
        "sliding_norm": "sliding_norm",
        "mass": "mass",
        "cog": "cog",
        "inertia_principal": "inertia_principal",
        "inertia_off_diagonal": "inertia_off_diagonal",
    }
    for key, (primary, secondary, group_labels) in series.items():
        axis = axis_groups[key]
        axis.clear()
        axis.set_title(labels[key][0])
        axis.grid(True, alpha=0.25)
        names = labels[key][1]
        for column in range(primary.shape[1]):
            label = f"{group_labels[0]} {names[column]}"
            axis.plot(
                times,
                primary[:, column],
                color=color_for_series(label, column),
                linestyle=line_style_for_series(label),
                linewidth=1.4,
                label=label,
            )
        if secondary is not None:
            for column in range(secondary.shape[1]):
                label = f"{group_labels[1]} {names[column]}"
                axis.plot(
                    times,
                    secondary[:, column],
                    color=color_for_series(label, column),
                    linestyle=line_style_for_series(label),
                    alpha=0.8,
                    linewidth=1.4,
                    label=label,
                )
        for item in history:
            if item.event:
                axis.axvline(item.t, color="tab:red", alpha=0.5, linewidth=1.0)
        axis.legend(loc="upper left", fontsize=7, ncol=2)
        axis.set_xlim(times[0], max(times[-1], times[0] + 1e-3))
        axis.set_ylim(*limit_for(families[key]))
    axis_groups["inertia_off_diagonal"].set_xlabel("time (s)")


def _center_of_mass(parameters, np):
    """Convert pseudo-inertia first moments [h] into CoM coordinates."""
    mass = parameters[:, 0]
    safe_mass = np.where(np.abs(mass) > 1e-12, mass, np.nan)
    return parameters[:, 1:4] / safe_mass[:, None]


class LiveDashboard:
    """Non-blocking owner of the dashboard child process."""

    def __init__(self, title: str = "Visual Geometric Control — Live Diagnostics"):
        context = mp.get_context("spawn")
        self._queue = context.Queue(maxsize=2)
        self._stop_event = context.Event()
        self._process = context.Process(
            target=_run_dashboard,
            args=(self._queue, self._stop_event, title),
            daemon=True,
        )
        self._process.start()

    @property
    def alive(self) -> bool:
        return self._process.is_alive()

    def publish(self, snapshot: VisualizationSnapshot) -> bool:
        """Publish newest snapshot, discarding stale frames without blocking."""
        try:
            self._queue.put_nowait(snapshot)
            return True
        except queue.Full:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._queue.put_nowait(snapshot)
                return True
            except queue.Full:
                return False

    def close(self) -> None:
        if not self._process.is_alive():
            return
        self._stop_event.set()
        self._process.join(timeout=2.0)
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=1.0)
