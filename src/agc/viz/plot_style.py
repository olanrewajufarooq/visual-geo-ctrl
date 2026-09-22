"""Shared publication plotting policy for time-series and trajectory figures."""

from __future__ import annotations

from typing import Any, Dict, Iterable, Optional, Sequence, Tuple

import numpy as np

COLOR_BLACK = "k"
COMPONENT_COLORS = ("r", "g", "b")
COMPARISON_COLORS = {"c1": "#E69F00", "euclidean": "#E69F00", "c2": "#0072B2", "bregman": "#0072B2"}
SIGNED_FAMILIES = {
    "position", "orientation", "linear_velocity", "angular_velocity",
    "force", "torque", "linear_acceleration", "angular_acceleration",
}

# Fixed limits are shared across figures so controller comparisons remain
# directly comparable. They encode the intended experiment viewing envelope.
LIMITS: Dict[str, Tuple[Any, Any]] = {
    "altitude": (0.0, 1.2),
    "position": (-7.5, 7.5),
    "orientation": (-180.0, 180.0),
    "linear_velocity": (-10.0, 10.0),
    "angular_velocity": (-5.0, 5.0),
    "linear_acceleration": (-100.0, 100.0),
    "angular_acceleration": (-50.0, 50.0),
    "force": (-125.0, 125.0),
    "torque": (-2.0, 2.0),
    "position_error": (0.0, 2.0),
    "attitude_error": (0.0, 180.0),
    "sliding_norm": (0.0, 2.0),
    "wrench_norm": (0.0, 125.0),
    "transverse_energy": (0.0, 5.0),
    "parameter_error": (0.0, 0.5),
    "pseudo_margin": (0.0, 0.02),
    "potential": (0.0, 0.5),
    "mass": (2.5, 5.0),
    "cog": (-0.025, 0.04),
    "inertia_principal": (0.03, 0.12),
    "inertia_off_diagonal": (-0.01, 0.02),
    "trajectory_x": (-7.5, 7.5),
    "trajectory_y": (-2.0, 2.0),
    "trajectory_z": (0.0, 1.2),
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
    """Return a stable RGB component color; reserve black for neutral events."""
    normalized = str(label).lower()
    if any(token in normalized for token in ("payload release", "simulation terminated", "failure")):
        return COLOR_BLACK
    for comparison, color in COMPARISON_COLORS.items():
        if normalized == comparison:
            return color
    return COMPONENT_COLORS[component % len(COMPONENT_COLORS)]


def line_style_for_series(label: str) -> str:
    """Encode state by line style while preserving the component color."""
    normalized = str(label).lower()
    if "payload release" in normalized:
        return ":"
    if normalized in {"c2", "bregman"}:
        return "--"
    if any(token in normalized for token in ("desired", "true", "reference")):
        return "--"
    return "-"


def nice_limits(
    series: Iterable[Sequence[float]],
    *,
    include_zero: bool,
    padding: float = 0.10,
) -> Tuple[float, float]:
    """Return data-driven, 1-2-5-rounded limits for the supplied traces."""
    arrays = [np.asarray(values, dtype=float).ravel() for values in series]
    finite = [values[np.isfinite(values)] for values in arrays]
    usable = [values for values in finite if values.size]
    values = np.concatenate(usable) if usable else np.array([0.0])
    lower = float(np.min(values))
    upper = float(np.max(values))
    if include_zero:
        lower = min(lower, 0.0)
        upper = max(upper, 0.0)
    span = upper - lower
    if span < 1e-12:
        span = 0.2 * max(abs(lower), abs(upper), 1.0)
        lower -= 0.5 * span
        upper += 0.5 * span
    else:
        lower -= padding * span
        upper += padding * span
    if include_zero:
        bound = max(abs(lower), abs(upper))
        step = _nice_step(bound / 3.0)
        rounded_bound = np.ceil(bound / step) * step
        return -float(rounded_bound), float(rounded_bound)
    step = _nice_step((upper - lower) / 5.0)
    return float(np.floor(lower / step) * step), float(np.ceil(upper / step) * step)


def _nice_step(value: float) -> float:
    """Choose a 1-2-5 tick interval at or above the requested scale."""
    magnitude = max(float(value), 1e-12)
    base = 10.0 ** np.floor(np.log10(magnitude))
    steps = np.array([1.0, 2.0, 5.0, 10.0])
    return float(steps[np.searchsorted(steps, magnitude / base, side="left")] * base)


def _component_index(label: str, ordinal: int, family: str) -> int:
    """Pair actual/reference channels by their shared ordinal component."""
    normalized = str(label).lower()
    if any(token in normalized for token in ("payload release", "failure", "terminated")):
        return 0
    if normalized in {"actual", "desired", "reference", "estimate", "true"}:
        return 0
    return ordinal % len(COMPONENT_COLORS)


def wrap_degrees(values: Sequence[float]) -> np.ndarray:
    """Wrap angles to the half-open display interval [-180, 180)."""
    return (np.asarray(values, dtype=float) + 180.0) % 360.0 - 180.0


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
    """Plot series with common colors, fixed limits, and failure signaling."""
    series = tuple(series)
    t_values = np.asarray(t, dtype=float) - float(x_offset)
    plotted = []
    clipped_count = 0
    for component, (values, label) in enumerate(zip(series, labels)):
        component_index = _component_index(label, component, family)
        plotted_values = np.asarray(values, dtype=float)
        color = color_for_series(label, component_index)
        line, = ax.plot(
            t_values,
            plotted_values,
            color=color,
            linestyle=line_style_for_series(label),
            linewidth=1.4,
            label=label,
        )
        plotted.append(line)
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
