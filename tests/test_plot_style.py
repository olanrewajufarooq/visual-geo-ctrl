"""Tests for publication plotting policy and display transforms."""

import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from agc.viz.plot_style import (  # noqa: E402
    COLOR_BLACK,
    COMPONENT_COLORS,
    LIMITS,
    clip_for_display,
    get_time_horizon,
)


def test_time_horizon_uses_requested_duration_and_logged_data_stays_short():
    run = {
        "t": np.array([0.0, 0.5, 1.0]),
        "metadata": {"duration": 2.0},
    }

    start, end = get_time_horizon(run)

    assert (start, end) == (0.0, 2.0)


def test_release_relative_horizon_uses_requested_duration():
    run = {
        "t": np.array([0.0, 0.5, 1.0]),
        "metadata": {"duration": 2.0},
    }

    assert get_time_horizon(run, release_time=0.75) == (0.0, 1.25)


def test_clip_for_display_preserves_source_and_reports_out_of_bounds():
    source = np.array([-3.0, -1.0, 0.5, 4.0])

    clipped, mask = clip_for_display(source, "position_error")

    assert np.array_equal(source, [-3.0, -1.0, 0.5, 4.0])
    assert np.array_equal(clipped, [0.0, 0.0, 0.5, 2.0])
    assert np.array_equal(mask, [True, True, False, True])
    assert LIMITS["position_error"] == (0.0, 2.0)


def test_shared_palette_is_rgb_and_black_only():
    assert COMPONENT_COLORS == ("r", "g", "b")
    assert COLOR_BLACK == "k"
