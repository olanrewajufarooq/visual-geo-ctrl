"""Regression tests for Python manual gains matching the MATLAB registry."""

import numpy as np
import pytest

from agc.config.manual_gains import manual_gains


BASELINE = {
    "KRdiag": np.array([0.01, 0.01, 2.0]),
    "Kxidiag": np.array([0.4, 0.4, 0.4]),
    "LambdaDiag": np.array([0.1, 0.1, 0.1, 0.1638, 0.2499, 0.1605]),
    "kd": 1.5,
    "ks": 1.0,
    "alpha": 0.5,
    "gammaE": np.full(10, 0.001),
    "gammaB": 0.001,
}


def assert_gains_equal(actual, expected):
    for field, expected_value in expected.items():
        if isinstance(expected_value, np.ndarray):
            np.testing.assert_array_equal(actual[field], expected_value)
        else:
            assert actual[field] == expected_value


def test_manual_gains_match_matlab_registry_for_all_scenarios():
    expected = {
        "nominal_lc": BASELINE,
        "nominal_rb": {
            **BASELINE,
            "KRdiag": np.array([100.0, 100.0, 200.0]),
            "Kxidiag": np.array([5.0, 5.0, 5.0]),
            "LambdaDiag": np.array([100.0, 100.0, 100.0, 10.0, 10.0, 10.0]),
        },
        "euclidean_lc": BASELINE,
        "euclidean_rb": BASELINE,
        "bregman_lc": BASELINE,
        "bregman_rb": BASELINE,
    }

    for key, expected_gains in expected.items():
        mode, coriolis = key.split("_")
        assert_gains_equal(manual_gains(mode, coriolis), expected_gains)


def test_manual_gains_reject_unknown_scenarios():
    with pytest.raises(KeyError, match="Unknown scenario key"):
        manual_gains("unknown", "lc")

    with pytest.raises(KeyError, match="Unknown scenario key"):
        manual_gains("bregman", "xyz")


def test_manual_gains_return_independent_values():
    first = manual_gains("nominal", "lc")
    second = manual_gains("nominal", "lc")

    first["KRdiag"][0] = 999.0
    first["gammaE"][0] = 999.0

    assert second["KRdiag"][0] == BASELINE["KRdiag"][0]
    assert second["gammaE"][0] == BASELINE["gammaE"][0]
