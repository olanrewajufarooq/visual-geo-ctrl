"""Shared publication plotting policy for time-series and trajectory figures."""

from __future__ import annotations

from typing import Any, Dict, Iterable, Optional, Sequence, Tuple

import numpy as np

COLOR_BLACK = "k"
COMPONENT_COLORS = ("r", "g", "b")

# Limits are deliberately rounded and shared across figures so that controller
# comparisons remain visually comparable.  Position uses per-axis limits.
LIMITS: Dict[str, Tuple[Any, Any]] = {
    "position": (np.array([-12.0, -4.0, -6.0]), np.array([12.0, 4.0, 6.0])),
    "orientation": (-180.0, 180.0),
    "linear_velocity": (-20.0, 20.0),
    "angular_velocity": (-10.0, 10.0),
    "linear_acceleration": (-100.0, 100.0),
    "angular_acceleration": (-50.0, 50.0),
    "force": (-1000.0, 1000.0),
    "torque": (-250.0, 250.0),
    "position_error": (0.0, 2.0),
    "attitude_error": (0.0, 180.0),
    "sliding_norm": (0.0, 50.0),
    "wrench_norm": (0.0, 1000.0),
    "transverse_energy": (0.0, 100.0),
    "parameter_error": (0.0, 1.0),
    "pseudo_margin": (0.0, 0.02),
    "potential": (0.0, 10.0),
    "mass": (0.0, 20.0),
    "cog": (-1.0, 1.0),
    "inertia": (-2.0, 2.0),
}


def get_time_horizon(run: Dict[str, Any], release_time: Optional[float] = None) -> Tuple[float, float]:
    """Return the intended plot horizon, independent of logged sample count."""
    metadata = run.get("metadata", {}) or {}
    duration = metadata.get("duration")
    if duration is None:
        t = np.asarray(run["t"], dtype=float)
        duration = float(t[-1]) if t.size else 0.0
    duration = max(0.0, float(duration))
    if release_time is None:
        return 0.0, duration
    return 0.0, max(0.0, duration - float(release_time))


def limit_for(family: str, component: Optional[int] = None) -> Tuple[float, float]:
    """Return scalar lower/upper display limits for a signal family."""
    lower, upper = LIMITS[family]
    if component is not None and np.ndim(lower):
        return float(np.asarray(lower)[component]), float(np.asarray(upper)[component])
    if np.ndim(lower):
        return float(np.min(lower)), float(np.max(upper))
    return float(lower), float(upper)


def clip_for_display(values: Sequence[float], family: str, component: Optional[int] = None):
    """Clip display values and return ``(clipped, out_of_bounds_mask)``.

    The input is never modified. Non-finite values remain non-finite and are
    not counted as out-of-bounds; callers may use them to create line gaps.
    """
    lower, upper = limit_for(family, component)
    source = np.asarray(values, dtype=float)
    finite = np.isfinite(source)
    out = finite & ((source < lower) | (source > upper))
    return np.clip(source, lower, upper), out


def color_for_series(label: str, component: int = 0) -> str:
    """Use black for references/ground truth and RGB for measured components."""
    normalized = str(label).lower()
    if any(token in normalized for token in ("desired", "true", "reference", "payload release")):
        return COLOR_BLACK
    return COMPONENT_COLORS[component % len(COMPONENT_COLORS)]


def _component_index(label: str, ordinal: int, family: str) -> Optional[int]:
    if family not in {"position"}:
        return None
    normalized = str(label).lower()
    for token, component in (("x", 0), ("y", 1), ("z", 2)):
        if normalized == token or normalized.startswith(token + " ") or normalized.startswith(token + "_"):
            return component
    return 2


def plot_time_series(
    ax,
    t: Sequence[float],
    series: Iterable[Sequence[float]],
    labels: Sequence[str],
    family: str,
    horizon: Tuple[float, float],
    *,
    x_offset: float = 0.0,
    failure_time: Optional[float] = None,
    release: Optional[float] = None,
):
    """Plot clipped series with common colors, limits, and failure signaling."""
    t_values = np.asarray(t, dtype=float) - float(x_offset)
    plotted = []
    clipped_count = 0
    for component, (values, label) in enumerate(zip(series, labels)):
        component_index = _component_index(label, component, family)
        clipped, out = clip_for_display(values, family, component_index)
        color = color_for_series(label, component)
        line, = ax.plot(t_values, clipped, color=color, linewidth=1.2, label=label)
        plotted.append(line)
        clipped_count += int(np.count_nonzero(out))
        if np.any(out):
            upper = np.isfinite(np.asarray(values, dtype=float)) & (np.asarray(values, dtype=float) > limit_for(family, component_index)[1])
            lower = np.isfinite(np.asarray(values, dtype=float)) & (np.asarray(values, dtype=float) < limit_for(family, component_index)[0])
            ax.plot(t_values[upper], clipped[upper], linestyle="", marker="^", color=color, markersize=3)
            ax.plot(t_values[lower], clipped[lower], linestyle="", marker="v", color=color, markersize=3)

    lower, upper = limit_for(family)
    ax.set_ylim(lower, upper)
    ax.set_xlim(*horizon)
    ax.margins(x=0)
    if release is not None and horizon[0] <= release <= horizon[1]:
        ax.axvline(release, color=COLOR_BLACK, linestyle=":", label="payload release")

    logged_end = float(t_values[-1]) if t_values.size else horizon[0]
    if logged_end < horizon[1] - 1e-12:
        ax.axvspan(logged_end, horizon[1], color=COLOR_BLACK, alpha=0.08, label="unlogged interval")
    if failure_time is not None and horizon[0] <= failure_time <= horizon[1]:
        ax.axvline(failure_time, color=COLOR_BLACK, linestyle="--", label="simulation terminated")
    if clipped_count:
        ax.plot([], [], color=COLOR_BLACK, marker="^", linestyle="", label=f"out of bounds ({clipped_count})")
    return plotted, clipped_count
