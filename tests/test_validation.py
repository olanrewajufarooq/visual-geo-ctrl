"""Unit tests for scenario and trajectory validation (Task 3)."""

import pytest
import numpy as np

from agc.sim.default_scenario import default_scenario
from agc.sim.validation import validate_scenario, validate_replay_data


def test_valid_default_scenarios_pass_validation():
    """Verify that all default scenario variants pass validation cleanly."""
    for mode in ["nominal", "euclidean", "bregman"]:
        for coriolis in ["lc", "rb"]:
            scen = default_scenario(
                replay_id="lemniscate_01_auto",
                mode=mode,
                coriolis=coriolis,
                duration=1.0,
            )
            validate_scenario(scen)


def test_missing_required_keys_raises_key_error():
    """Verify KeyError when required scenario keys are missing."""
    scen = default_scenario(duration=1.0)
    del scen["duration"]
    with pytest.raises(KeyError, match="missing required key: 'duration'"):
        validate_scenario(scen)


def test_non_integer_control_ratio_raises_value_error():
    """Verify error when dtControl is not an integer multiple of dtPlant."""
    scen = default_scenario(duration=1.0)
    scen["dtPlant"] = 0.002
    scen["dtControl"] = 0.005  # 0.005 / 0.002 = 2.5 (non-integer)
    with pytest.raises(ValueError, match="integer multiple"):
        validate_scenario(scen)


def test_invalid_controller_mode_raises_value_error():
    """Verify error on unsupported controller mode."""
    scen = default_scenario(duration=1.0)
    scen["controller"]["mode"] = "neural_network"
    with pytest.raises(ValueError, match="Invalid controller mode"):
        validate_scenario(scen)


def test_invalid_coriolis_form_raises_value_error():
    """Verify error on unsupported Coriolis form (including dropped c1/c2)."""
    scen = default_scenario(duration=1.0)
    scen["controller"]["coriolis"] = "c1"
    with pytest.raises(ValueError, match="Invalid Coriolis factorization"):
        validate_scenario(scen)


def test_non_spd_gain_raises_value_error():
    """Verify error when a gain matrix is not positive definite."""
    scen = default_scenario(duration=1.0)
    scen["controller"]["KR"] = np.diag([10.0, -2.0, 10.0])  # Negative eigenvalue
    with pytest.raises(ValueError, match="symmetric positive definite"):
        validate_scenario(scen)


def test_lambda_must_be_a_6x6_symmetric_positive_definite_matrix():
    """Lambda is a six-dimensional sliding gain, unlike KR and Kxi."""
    scen = default_scenario(duration=1.0)

    scen["controller"]["Lambda"] = np.eye(3)
    with pytest.raises(ValueError, match="Lambda must be a 6x6 symmetric positive definite matrix"):
        validate_scenario(scen)

    scen["controller"]["Lambda"] = np.diag([1.0, 1.0, 1.0, 1.0, 1.0, -1.0])
    with pytest.raises(ValueError, match="Lambda must be a 6x6 symmetric positive definite matrix"):
        validate_scenario(scen)


def test_payload_release_boundary_alignment():
    """Verify error if releaseTime is <= 0 or unaligned."""
    scen = default_scenario(duration=5.0)
    # releaseTime <= 0
    scen["payloadDrop"]["releaseTime"] = -1.0
    with pytest.raises(ValueError, match="strictly positive"):
        validate_scenario(scen)

    # releaseTime unaligned with dtPlant
    scen["dtPlant"] = 0.002
    scen["payloadDrop"]["releaseTime"] = 2.0005
    with pytest.raises(ValueError, match="aligned to dtPlant"):
        validate_scenario(scen)


def test_replay_data_validation():
    """Verify validate_replay_data checks sample count, monotonicity, and orientation."""
    n = 10
    t = np.linspace(0.0, 1.0, n)
    good_data = {
        "t": t,
        "p": np.zeros((n, 3)),
        "v_b": np.zeros((n, 3)),
        "a_b": np.zeros((n, 3)),
        "omega_b": np.zeros((n, 3)),
        "alpha_b": np.zeros((n, 3)),
        "R": np.tile(np.eye(3)[:, :, None], (1, 1, n)),
    }
    validate_replay_data(good_data)

    # Non-monotonic time
    bad_t = np.copy(t)
    bad_t[3] = bad_t[2]  # not strictly increasing
    bad_data = dict(good_data, t=bad_t)
    with pytest.raises(ValueError, match="monotonically increasing"):
        validate_replay_data(bad_data)

    # Mismatched lengths
    bad_p = np.zeros((n - 2, 3))
    bad_data2 = dict(good_data, p=bad_p)
    with pytest.raises(ValueError, match="shape"):
        validate_replay_data(bad_data2)
