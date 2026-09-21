"""Tests for paper equations and controller core, matching MATLAB TestPaperCore.m."""

import sys
from pathlib import Path
import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from agc.math.se3 import skew, unskew, ad_twist, expm_so3
from agc.math.inertia import inertia_from_pi, pseudo_from_pi, is_spd
from agc.paper.coriolis import coriolis
from agc.paper.errors import potential, potential_derivative
from agc.paper.regressor import regressor
from agc.paper.adaptation import bregman_step, euclidean_step
from agc.paper.controller import controller


def test_c1_and_c2_agree_on_body_velocity():
    pi = np.array([2.4, 0.12, -0.07, 0.04, 0.22, 0.31, 0.38, 0.01, -0.02, 0.03])
    I6 = inertia_from_pi(pi)
    V = np.array([0.7, -0.3, 0.2, 1.1, -0.4, 0.6])

    c1 = coriolis("c1", V, I6, V)
    c2 = coriolis("c2", V, I6, V)
    assert np.allclose(c1, c2, atol=1e-12)


def test_c1_uses_levi_civita_factorization():
    pi = np.array([1.9, 0.08, 0.03, -0.05, 0.19, 0.27, 0.33, 0.02, 0.01, -0.015])
    I6 = inertia_from_pi(pi)
    V = np.array([0.4, -0.2, 0.5, 0.7, 0.1, -0.3])
    U = np.array([-0.6, 0.8, 0.1, -0.2, 0.9, 0.3])

    adV = ad_twist(V)
    adU = ad_twist(U)
    expected = 0.5 * (I6 @ adV @ U - adU.T @ (I6 @ V) - adV.T @ (I6 @ U))
    actual = coriolis("c1", V, I6, U)
    assert np.allclose(actual, expected, atol=1e-12)


def test_c2_uses_alternative_coadjoint_factorization():
    pi = np.array([1.9, 0.08, 0.03, -0.05, 0.19, 0.27, 0.33, 0.02, 0.01, -0.015])
    I6 = inertia_from_pi(pi)
    V = np.array([0.4, -0.2, 0.5, 0.7, 0.1, -0.3])
    U = np.array([-0.6, 0.8, 0.1, -0.2, 0.9, 0.3])

    adU = ad_twist(U)
    expected = -adU.T @ (I6 @ V)
    actual = coriolis("c2", V, I6, U)
    assert np.allclose(actual, expected, atol=1e-12)


def test_error_covector_uses_paper_attitude_scale():
    theta = np.pi / 3.0
    R = np.array([
        [np.cos(theta), -np.sin(theta), 0.0],
        [np.sin(theta),  np.cos(theta), 0.0],
        [0.0,            0.0,           1.0]
    ])
    He = np.eye(4)
    He[0:3, 0:3] = R
    He[0:3, 3] = np.array([0.3, -0.2, 0.1])
    KR = np.diag([2.0, 3.0, 5.0])
    Kxi = np.diag([7.0, 11.0, 13.0])

    expected_eR = unskew(0.5 * (KR @ R - R.T @ KR))
    expected_ep = R.T @ Kxi @ He[0:3, 3]
    expected_eH = np.concatenate([expected_eR, expected_ep])

    eH, psi = potential(He, KR, Kxi)
    assert np.allclose(eH, expected_eH, atol=1e-12)
    assert psi > 0.0


def test_bregman_step_preserves_positive_definiteness():
    pi = np.array([3.0, 0.15, -0.1, 0.08, 0.42, 0.51, 0.62, 0.01, -0.03, 0.02])
    J = pseudo_from_pi(pi)
    G = np.array([
        [2.0, -1.0,  0.0,  3.0],
        [-1.0, -4.0,  2.0,  0.0],
        [0.0,   2.0,  1.0, -2.0],
        [3.0,   0.0, -2.0,  5.0]
    ])

    J_next = bregman_step(J, G, gamma=50.0, dt=0.2)
    assert is_spd(J_next)
    assert np.allclose(J_next, J_next.T, atol=1e-12)


def test_euclidean_step_matches_gradient_update():
    pi_hat = np.array([2.0, 0.1, -0.2, 0.3, 0.4, 0.5, 0.6, 0.01, -0.02, 0.03])
    gradient = np.array([0.2, -0.3, 0.1, 0.4, -0.5, 0.6, -0.7, 0.8, -0.9, 1.0])
    Gamma = np.diag(np.arange(1, 11) * 1e-2)
    dt = 0.05

    actual = euclidean_step(pi_hat, gradient, Gamma, dt)
    expected = pi_hat - dt * (Gamma @ gradient)
    assert np.allclose(actual, expected, atol=1e-12)


def test_regressor_matches_both_paper_factorizations():
    pi = np.array([2.1, 0.05, -0.03, 0.02, 0.33, 0.41, 0.49, 0.01, 0.02, -0.01])
    H = np.eye(4)
    axis = np.array([0.2, -0.3, 0.4])
    angle = 0.6
    H[0:3, 0:3] = expm_so3((axis / np.linalg.norm(axis)) * angle)

    V = np.array([0.5, -0.4, 0.1, 0.6, 0.2, -0.7])
    Vr = np.array([-0.2, 0.3, 0.4, 0.1, -0.5, 0.6])
    Vrdot = np.array([0.7, 0.2, -0.1, -0.3, 0.4, 0.5])
    I6 = inertia_from_pi(pi)
    g = np.array([0.0, 0.0, 9.81])

    for form in ["c1", "c2"]:
        Y = regressor(H, V, Vr, Vrdot, g, form)
        g_body = H[0:3, 0:3].T @ g
        Wg = np.concatenate([skew(pi[1:4]) @ g_body, pi[0] * g_body])
        adV = ad_twist(V)
        adVr = ad_twist(Vr)
        if form == "c1":
            coriolis_wrench = 0.5 * (I6 @ adV @ Vr - adVr.T @ (I6 @ V) - adV.T @ (I6 @ Vr))
        else:
            coriolis_wrench = -adVr.T @ (I6 @ V)
        expected = I6 @ Vrdot + coriolis_wrench + Wg
        assert np.allclose(Y @ pi, expected, atol=1e-11)


def test_nominal_controller_uses_fractional_dissipation():
    state = {"H": np.eye(4), "V": np.array([0.0, 0.0, 0.0, 0.4, 0.0, 0.0])}
    desired = {"H": np.eye(4), "V": np.zeros(6), "Vdot": np.zeros(6)}
    pi = np.array([1.5, 0.0, 0.0, 0.0, 0.2, 0.25, 0.3, 0.0, 0.0, 0.0])
    cfg = {
        "mode": "nominal",
        "coriolis": "c1",
        "KR": np.eye(3),
        "Kxi": np.eye(3),
        "Lambda": np.eye(6),
        "kd": 1.0,
        "ks": 2.0,
        "alpha": 0.5,
        "gravity": np.array([0.0, 0.0, 9.81]),
    }

    W, diagnostics, _ = controller(state, desired, cfg, pi, dt_adapt=None)

    s_norm = np.linalg.norm(diagnostics.s)
    expected_D = (1.0 + 2.0 * (s_norm ** (-0.5))) * diagnostics.s
    I6 = inertia_from_pi(pi)
    expected = (
        I6 @ diagnostics.VrDot
        + coriolis("c1", state["V"], I6, diagnostics.Vr)
        + diagnostics.Wg
        - expected_D
    )
    assert np.allclose(W, expected, atol=1e-12)


def test_potential_derivative_matches_error_kinematics():
    axis = np.array([0.3, 0.2, -0.1])
    R0 = expm_so3((axis / np.linalg.norm(axis)) * 0.4)
    He = np.eye(4)
    He[0:3, 0:3] = R0
    He[0:3, 3] = np.array([0.2, -0.1, 0.3])
    Ve = np.array([0.4, -0.5, 0.2, 0.3, 0.1, -0.2])

    KR = np.diag([2.0, 3.0, 4.0])
    Kxi = np.diag([5.0, 6.0, 7.0])
    dt = 1e-7

    e0, _ = potential(He, KR, Kxi)
    R_next = He[0:3, 0:3] @ expm_so3(Ve[0:3] * dt)
    p_next = He[0:3, 3] + He[0:3, 0:3] @ Ve[3:6] * dt
    H_next = np.eye(4)
    H_next[0:3, 0:3] = R_next
    H_next[0:3, 3] = p_next

    e1, _ = potential(H_next, KR, Kxi)
    derivative = potential_derivative(He, Ve, KR, Kxi)
    fd_derivative = (e1 - e0) / dt
    assert np.allclose(derivative, fd_derivative, atol=1e-5)


def test_controller_zero_error_behavior():
    """Verify that when state == desired at hover (V = 0), tracking error and sliding
    vector are zero, and the commanded wrench exactly balances gravity."""
    H = np.eye(4)
    V = np.zeros(6)
    state = {"H": H, "V": V}
    desired = {"H": H, "V": V, "Vdot": np.zeros(6)}

    pi = np.array([2.5, 0.05, -0.02, 0.01, 0.3, 0.35, 0.4, 0.0, 0.0, 0.0])
    g = np.array([0.0, 0.0, 9.81])
    cfg = {
        "mode": "nominal",
        "coriolis": "c1",
        "KR": np.eye(3) * 5.0,
        "Kxi": np.eye(3) * 10.0,
        "Lambda": np.eye(6) * 2.0,
        "kd": 1.0,
        "ks": 1.0,
        "alpha": 0.8,
        "gravity": g,
    }

    W, diag, _ = controller(state, desired, cfg, pi, dt_adapt=None)
    assert np.allclose(diag.s, np.zeros(6), atol=1e-12)
    assert abs(diag.Psi) < 1e-12
    assert np.allclose(diag.eH, np.zeros(6), atol=1e-12)

    # Command wrench should equal rigid body gravity wrench Wg
    assert np.allclose(W, diag.Wg, atol=1e-12)


def test_controller_all_modes_and_factorizations():
    """Verify controller execution and adaptation update across all 6 paper variants."""
    H = np.eye(4)
    state = {"H": H, "V": np.array([0.2, -0.1, 0.1, 0.5, -0.3, 0.2])}
    desired = {"H": H, "V": np.zeros(6), "Vdot": np.zeros(6)}

    pi = np.array([3.0, 0.05, -0.02, 0.01, 0.4, 0.45, 0.5, 0.01, -0.01, 0.02])
    g = np.array([0.0, 0.0, 9.81])

    for mode in ["nominal", "euclidean", "bregman"]:
        for form in ["c1", "c2"]:
            cfg = {
                "mode": mode,
                "coriolis": form,
                "KR": np.eye(3) * 6.0,
                "Kxi": np.eye(3) * 12.0,
                "Lambda": np.eye(6) * 1.5,
                "kd": 1.2,
                "ks": 0.8,
                "alpha": 0.7,
                "gravity": g,
                "gammaE": 0.01 * np.ones(10),
                "gammaB": 0.05,
            }
            est = pseudo_from_pi(pi) if mode == "bregman" else np.copy(pi)

            W, diag, next_est = controller(state, desired, cfg, est, dt_adapt=0.01)
            assert len(W) == 6 and np.all(np.isfinite(W))
            assert len(diag.s) == 6
            assert diag.Vs >= 0.0

            if mode == "bregman":
                assert is_spd(next_est)
            else:
                assert next_est[0] > 0.0


def test_invalid_controller_mode_raises():
    """Verify ValueError for unsupported mode or coriolis form."""
    state = {"H": np.eye(4), "V": np.zeros(6)}
    desired = {"H": np.eye(4), "V": np.zeros(6), "Vdot": np.zeros(6)}
    pi = np.array([2.0, 0, 0, 0, 0.2, 0.2, 0.2, 0, 0, 0])
    cfg = {
        "mode": "neural",
        "coriolis": "c1",
        "KR": np.eye(3),
        "Kxi": np.eye(3),
        "Lambda": np.eye(6),
        "kd": 1.0,
        "ks": 1.0,
        "alpha": 0.5,
        "gravity": np.array([0.0, 0.0, 9.81]),
    }
    with pytest.raises(ValueError, match="Unknown controller mode"):
        controller(state, desired, cfg, pi)

    cfg["mode"] = "nominal"
    cfg["coriolis"] = "c3"
    with pytest.raises(ValueError, match="Unknown form 'c3'"):
        controller(state, desired, cfg, pi)
