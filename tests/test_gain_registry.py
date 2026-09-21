"""Tests for manual and optimized gain registries and gain promotion."""

import tempfile
from pathlib import Path
import numpy as np
import pytest

from agc.config.manual_gains import manual_gains as agc_manual_gains
from agc.config.optimized_gains import optimized_gains as agc_optimized_gains
import config
from agc.opt.staged_optimizer import promote_gains_to_registry
from agc.math import is_spd


@pytest.mark.parametrize("mode", ["nominal", "euclidean", "bregman"])
@pytest.mark.parametrize("coriolis", ["c1", "c2"])
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


def test_root_config_reexports_match_package():
    for mode in ["nominal", "euclidean", "bregman"]:
        for coriolis in ["c1", "c2"]:
            pkg_m = agc_manual_gains(mode, coriolis)
            root_m = config.manual_gains(mode, coriolis)
            np.testing.assert_allclose(pkg_m["KRdiag"], root_m["KRdiag"])
            np.testing.assert_allclose(pkg_m["LambdaDiag"], root_m["LambdaDiag"])

            pkg_o = agc_optimized_gains(mode, coriolis)
            root_o = config.optimized_gains(mode, coriolis)
            np.testing.assert_allclose(pkg_o["KRdiag"], root_o["KRdiag"])
            np.testing.assert_allclose(pkg_o["LambdaDiag"], root_o["LambdaDiag"])


def test_gain_promotion_writes_valid_python(tmp_path):
    target = tmp_path / "optimized_gains.py"
    current = agc_optimized_gains("bregman", "c1")
    new_gains = dict(current)
    new_gains["kd"] = 2.4567

    promote_gains_to_registry(
        mode="bregman",
        coriolis="c1",
        gains=new_gains,
        metadata={"stage": "test", "cost": 12.34},
        target_file=str(target),
    )

    assert target.exists()
    content = target.read_text(encoding="utf-8")
    assert "def optimized_gains" in content
    assert "2.457" in content or "2.4567" in content

    # Dynamically exec to verify python validity
    namespace = {}
    exec(content, namespace)
    loaded_func = namespace["optimized_gains"]
    loaded = loaded_func("bregman", "c1")
    assert abs(loaded["kd"] - 2.457) < 1e-2
