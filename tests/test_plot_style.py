"""Tests for publication plotting policy and display transforms."""

import sys
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from agc.viz.plot_style import (  # noqa: E402
    COLOR_BLACK,
    COMPONENT_COLORS,
    LIMITS,
    clip_for_display,
    color_for_series,
    get_time_horizon,
    line_style_for_series,
    nice_limits,
    plot_time_series,
    wrap_degrees,
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
    assert COMPONENT_COLORS == ("#FF0000", "#00FF00", "#0000FF")
    assert COLOR_BLACK == "k"


def test_reference_and_truth_keep_their_component_color_but_use_dashes():
    assert color_for_series("x desired", 0) == "#FF0000"
    assert color_for_series("y desired", 1) == "#00FF00"
    assert color_for_series("Izz true", 2) == "#0000FF"
    assert line_style_for_series("x") == "-"
    assert line_style_for_series("x desired") == "--"
    assert line_style_for_series("Izz true") == "--"
    assert color_for_series("payload release") == COLOR_BLACK


def test_nice_limits_tighten_signed_force_range_and_keep_zero_visible():
    lower, upper = nice_limits([np.array([-125.0, 0.0, 110.0])], include_zero=True)

    assert lower == -150.0
    assert upper == 150.0


def test_nice_limits_tighten_small_positive_inertia_without_zero_centering():
    lower, upper = nice_limits([np.array([0.08, 0.10, 0.11])], include_zero=False)

    assert 0.0 < lower < 0.08
    assert 0.11 < upper < 0.2


def test_nice_limits_gives_flat_series_a_nonzero_range():
    lower, upper = nice_limits([np.array([0.1, 0.1])], include_zero=False)

    assert lower < 0.1 < upper


def test_time_plot_pairs_reference_with_component_color_and_preferred_bounds():
    figure, axis = plt.subplots()
    values = [
        np.array([-125.0, 0.0, 110.0]),
        np.array([-20.0, 0.0, 10.0]),
        np.array([30.0, 32.0, 31.0]),
        np.array([-120.0, 0.0, 105.0]),
        np.array([-18.0, 0.0, 8.0]),
        np.array([29.0, 31.0, 30.0]),
    ]
    lines, _ = plot_time_series(
        axis,
        [0.0, 1.0, 2.0],
        values,
        ["x", "y", "z", "x desired", "y desired", "z desired"],
        "force",
        (0.0, 2.0),
    )

    assert [line.get_color() for line in lines] == ["#FF0000", "#00FF00", "#0000FF"]*2
    assert [line.get_linestyle() for line in lines] == ["-", "-", "-", "--", "--", "--"]
    assert axis.get_ylim() == (-125.0, 125.0)
    plt.close(figure)


def test_preferred_bounds_distinguish_altitude_and_inertia_families():
    figure, (altitude_axis, principal_axis, off_diagonal_axis) = plt.subplots(3, 1)

    altitude_lines, _ = plot_time_series(altitude_axis, [0.0, 1.0], [[0.1, 0.9], [0.2, 0.8]], ["actual", "desired"], "altitude", (0.0, 1.0))
    plot_time_series(principal_axis, [0.0, 1.0], [[0.08, 0.1]], ["Ixx"], "inertia_principal", (0.0, 1.0))
    plot_time_series(off_diagonal_axis, [0.0, 1.0], [[-0.005, 0.01]], ["Ixy"], "inertia_off_diagonal", (0.0, 1.0))

    assert altitude_axis.get_ylim() == (0.0, 1.2)
    assert principal_axis.get_ylim() == (0.03, 0.12)
    assert off_diagonal_axis.get_ylim() == (-0.01, 0.02)
    assert [line.get_color() for line in altitude_lines] == ["#FF0000", "#FF0000"]
    assert [line.get_linestyle() for line in altitude_lines] == ["-", "--"]
    plt.close(figure)


def test_comparison_series_use_stable_distinct_colors_and_styles():
    figure, axis = plt.subplots()
    lines, _ = plot_time_series(
        axis,
        [0.0, 1.0],
        [[0.1, 0.2], [0.2, 0.1]],
        ["LC", "RB"],
        "sliding_norm",
        (0.0, 1.0),
    )

    assert [line.get_color() for line in lines] == ["#0000FF", "#FF0000"]
    assert [line.get_linestyle() for line in lines] == ["-", "--"]
    plt.close(figure)


def test_wrap_degrees_keeps_attitude_within_requested_display_interval():
    wrapped = wrap_degrees(np.array([-540.0, -181.0, -180.0, 0.0, 180.0, 181.0, 540.0]))

    assert np.array_equal(wrapped, np.array([-180.0, 179.0, -180.0, 0.0, -180.0, -179.0, -180.0]))
