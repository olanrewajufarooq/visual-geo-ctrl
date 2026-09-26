import numpy as np
import pytest
from agc.math import inertia
from agc.viz import publication_figures as figures


def test_central_moments_remove_parallel_axis_and_sort():
    # m=2, c=(1,0,0), rotated central tensor [[4,1,0],[1,4,0],[0,0,4]].
    pi = np.array([[2, 2, 0, 0, 4, 6, 6, 1, 0, 0]], dtype=float)
    center, moments = inertia.center_and_principal_moments(pi)
    np.testing.assert_allclose(center, [[1, 0, 0]])
    np.testing.assert_allclose(moments, [[3, 4, 5]])


def test_invalid_samples_leave_gaps_without_projecting_negative_moments():
    pi = np.array([[1, 0, 0, 0, -1, 2, 3, 0, 0, 0]] * 4, dtype=float)
    pi[1, 0] = 0
    pi[2, 0] = -1
    pi[3, 4] = np.nan
    with pytest.warns(RuntimeWarning):
        center, moments = inertia.center_and_principal_moments(pi)
    np.testing.assert_allclose(moments[0], [-1, 2, 3])
    assert np.isnan(center[1:]).all()
    assert np.isnan(moments[1:]).all()


def test_figures_export_piecewise_truth_and_physical_labels(tmp_path, monkeypatch):
    pi = np.array([[2, 2, 0, 0, 4, 6, 6, 1, 0, 0],
                   [1, 0, 0, 0, 3, 4, 5, 0, 0, 0]], dtype=float)
    run = {"t": np.array([0., 10.]), "estimatePi": pi, "activePlantPi": pi}
    captured = []
    original = figures.save
    def inspect(fig, out, name):
        captured.append((name, fig.axes[0].get_ylabel(), fig.axes[0].get_xlim(),
                         fig.axes[0].lines[-1].get_ydata().copy()))
        original(fig, out, name)
    monkeypatch.setattr(figures, "save", inspect)
    figures.inertial_estimate_figures({"euclidean": run, "bregman": run}, tmp_path)
    assert len(captured) == 2
    for name, label, limits, truth in captured:
        assert limits == (0, 30)
        assert "[m]" in label or "kg" in label
        for extension in ("pdf", "png"):
            assert (tmp_path / "02-physical-consistency" / f"{name}.{extension}").stat().st_size > 100
    np.testing.assert_allclose(captured[0][3], [1, 0])
    np.testing.assert_allclose(captured[1][3], [3, 3])
