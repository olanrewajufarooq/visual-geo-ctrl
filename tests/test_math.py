"""Tests for SE(3) and rigid-body inertia mathematics."""

import sys
from pathlib import Path
import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from agc.math.se3 import (
    skew,
    unskew,
    ad_twist,
    adjoint_se3,
    inv_se3,
    expm_so3,
    rotm_to_quat,
    quat_to_rotm,
)
from agc.math.inertia import (
    inertia_from_pi,
    pseudo_from_pi,
    pi_from_pseudo,
    is_spd,
)


def test_skew_unskew_roundtrip():
    v = np.array([1.5, -2.3, 4.1])
    S = skew(v)
    assert np.allclose(S + S.T, 0.0, atol=1e-12)
    v_rec = unskew(S)
    assert np.allclose(v, v_rec, atol=1e-12)


def test_skew_cross_product():
    v = np.array([0.5, -1.2, 2.0])
    y = np.array([1.1, 0.3, -0.7])
    expected = np.cross(v, y)
    actual = skew(v) @ y
    assert np.allclose(expected, actual, atol=1e-12)


def test_inv_se3():
    R = expm_so3(np.array([0.1, -0.2, 0.3]))
    p = np.array([1.0, 2.0, -0.5])
    H = np.eye(4)
    H[0:3, 0:3] = R
    H[0:3, 3] = p
    H_inv = inv_se3(H)
    assert np.allclose(H @ H_inv, np.eye(4), atol=1e-12)
    assert np.allclose(H_inv @ H, np.eye(4), atol=1e-12)


def test_rotm_quat_roundtrip():
    w = np.array([0.3, -0.5, 0.2])
    R = expm_so3(w)
    q = rotm_to_quat(R)
    R_rec = quat_to_rotm(q)
    assert np.allclose(R, R_rec, atol=1e-10)


def test_inertia_and_pseudo_inertia_roundtrip():
    # Physically consistent inertia parameter vector
    m = 3.5
    h = np.array([0.1, -0.05, 0.08])
    Ip = np.array([0.4, 0.5, 0.6, 0.01, -0.02, 0.03])
    pi = np.concatenate([[m], h, Ip])

    J = pseudo_from_pi(pi)
    assert is_spd(J)

    pi_rec = pi_from_pseudo(J)
    assert np.allclose(pi, pi_rec, atol=1e-12)


def test_adjoint_se3_structure():
    R = expm_so3(np.array([0.2, 0.1, -0.3]))
    p = np.array([0.5, -0.2, 0.8])
    H = np.eye(4)
    H[0:3, 0:3] = R
    H[0:3, 3] = p
    Ad = adjoint_se3(H)
    assert Ad.shape == (6, 6)
    assert np.allclose(Ad[0:3, 0:3], R, atol=1e-12)
    assert np.allclose(Ad[0:3, 3:6], np.zeros((3, 3)), atol=1e-12)
    assert np.allclose(Ad[3:6, 0:3], skew(p) @ R, atol=1e-12)
    assert np.allclose(Ad[3:6, 3:6], R, atol=1e-12)
