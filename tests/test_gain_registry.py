"""Tests for manual and optimized gain registries and gain promotion."""

import tempfile
from pathlib import Path
import numpy as np
import pytest

from agc.config.manual_gains import manual_gains as agc_manual_gains
from agc.config.optimized_gains import optimized_gains as agc_optimized_gains
from agc.opt.staged_optimizer import promote_gains_to_registry
from agc.math import is_spd


@pytest.mark.parametrize("mode", ["nominal", "euclidean", "bregman"])
@pytest.mark.parametrize("coriolis", ["lc", "rb"])
def test_manual_and_optimized_gains_have_matching_interface(mode, coriolis):
    mg = agc_manual_gains(mode, coriolis)
    og = agc_optimized_gains(mode, coriolis)

    for g in (mg, og):
        assert "KRdiag" in g and len(g["KRdiag"]) == 3
        assert "Kxidiag" in g and len(g["Kxidiag"]) == 3
        assert "LambdaDiag" in g and len(g["LambdaDiag"]) == 6
        assert "kd" in g and g["kd"] > 0
        assert "ks" in g and g["ks"] > 0
        assert "alpha" in g and g["alpha"] > 0
        assert "gammaE" in g and len(g["gammaE"]) == 10
        assert "gammaB" in g and g["gammaB"] > 0

        assert np.all(g["KRdiag"] > 0)
        assert np.all(g["Kxidiag"] > 0)
        assert np.all(g["LambdaDiag"] > 0)
        assert is_spd(np.diag(g["LambdaDiag"]))


def test_gain_promotion_writes_valid_python(tmp_path):
    target = tmp_path / "optimized_gains.py"
    current = agc_optimized_gains("bregman", "lc")
    new_gains = dict(current)
    new_gains["gammaB"] = 0.0024567

    promote_gains_to_registry(
        mode="bregman",
        coriolis="lc",
        gains=new_gains,
        metadata={"stage": "adaptive", "cost": 12.34},
        target_file=str(target),
    )

    assert target.exists()
    content = target.read_text(encoding="utf-8")
    assert "def optimized_gains" in content
    assert "0.002457" in content or "0.0024567" in content

    # Dynamically exec to verify python validity
    namespace = {}
    exec(content, namespace)
    loaded_func = namespace["optimized_gains"]
    loaded = loaded_func("bregman", "lc")
    assert abs(loaded["gammaB"] - 0.002457) < 1e-4
    assert loaded["optimizationCost"] == pytest.approx(12.34)


def test_optimized_registry_exposes_persisted_cost():
    gains = agc_optimized_gains("bregman", "rb")
    assert "optimizationCost" in gains
    assert np.isfinite(gains["optimizationCost"])


def test_optimized_gains_coriolis_invariance():
    """Optimized gains must be invariant to Coriolis factorization form (lc vs rb)."""
    modes = ["nominal", "euclidean", "bregman", "adaptive"]
    for m in modes:
        g_lc = agc_optimized_gains(m, "lc")
        g_rb = agc_optimized_gains(m, "rb")
        for k in ["KRdiag", "Kxidiag", "LambdaDiag"]:
            np.testing.assert_array_equal(g_lc[k], g_rb[k])
        for k in ["kd", "ks", "alpha"]:
            assert g_lc[k] == g_rb[k]
        if "gammaE" in g_lc:
            np.testing.assert_array_equal(g_lc["gammaE"], g_rb["gammaE"])
        if "gammaB" in g_lc:
            assert g_lc["gammaB"] == g_rb["gammaB"]


def test_shared_tracking_gains_across_all_modes():
    """All controller modes must share identical tracking and sliding gains."""
    tracking_keys = ["KRdiag", "Kxidiag", "LambdaDiag", "kd", "ks", "alpha"]
    modes = ["nominal", "euclidean", "bregman", "adaptive"]
    base = agc_optimized_gains("nominal", "lc")

    for m in modes:
        for f in ["lc", "rb"]:
            g = agc_optimized_gains(m, f)
            for k in ["KRdiag", "Kxidiag", "LambdaDiag"]:
                np.testing.assert_array_equal(g[k], base[k])
            for k in ["kd", "ks", "alpha"]:
                assert g[k] == base[k]


def test_gain_promotion_synchronizes_nominal_tracking_gains(tmp_path):
    """Promoting nominal gains updates tracking keys across all modes and Coriolis forms."""
    target = tmp_path / "optimized_gains_sync.py"
    current = agc_optimized_gains("nominal", "lc")
    new_gains = dict(current)
    new_gains["kd"] = 3.1415
    new_gains["ks"] = 1.6180

    promote_gains_to_registry(
        mode="nominal",
        coriolis="all",
        gains=new_gains,
        metadata={"stage": "all", "cost": 5.0},
        target_file=str(target),
    )

    namespace = {}
    exec(target.read_text(encoding="utf-8"), namespace)
    loaded_func = namespace["optimized_gains"]

    for m in ["nominal", "euclidean", "bregman"]:
        for f in ["lc", "rb"]:
            g = loaded_func(m, f)
            assert abs(g["kd"] - 3.142) < 1e-2 or abs(g["kd"] - 3.1415) < 1e-2
            assert abs(g["ks"] - 1.618) < 1e-2 or abs(g["ks"] - 1.6180) < 1e-2
