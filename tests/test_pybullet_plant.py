"""Tests for PyBullet simulation plant dynamics."""

import sys
from pathlib import Path
import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from agc.plant.pybullet_plant import PyBulletPlant
from agc.plant.compound_pi import compound_pi


def test_body_wrench_cancels_gravity_at_hover():
    pi = np.array([2.0, 0.0, 0.0, 0.0, 0.1, 0.12, 0.14, 0.0, 0.0, 0.0])
    gravity = np.array([0.0, 0.0, -9.81])
    dt = 0.002

    plant = PyBulletPlant(pi=pi, gravity=gravity, dt=dt, gui=False)
    state0 = {"H": np.eye(4), "V": np.zeros(6)}
    plant.set_state(state0["H"], state0["V"])

    # Wrench that exactly cancels gravity: upward force = m * 9.81
    hover_wrench = np.array([0.0, 0.0, 0.0, 0.0, 0.0, pi[0] * 9.81])

    # Step simulation
    for _ in range(50):
        plant.apply_wrench(hover_wrench)
        plant.step()

    state = plant.get_state()
    plant.close()

    # UAV should remain stationary at [0, 0, 0] with near-zero velocity
    assert np.allclose(state["H"][0:3, 3], np.zeros(3), atol=1e-4)
    assert np.allclose(state["V"], np.zeros(6), atol=1e-3)


def test_gravity_free_fall():
    pi = np.array([2.0, 0.0, 0.0, 0.0, 0.1, 0.12, 0.14, 0.0, 0.0, 0.0])
    gravity = np.array([0.0, 0.0, -9.81])
    dt = 0.002

    plant = PyBulletPlant(pi=pi, gravity=gravity, dt=dt, gui=False)
    state0 = {"H": np.eye(4), "V": np.zeros(6)}
    plant.set_state(state0["H"], state0["V"])

    # Step free fall for 100 steps = 0.2 s
    steps = 100
    t_total = steps * dt
    for _ in range(steps):
        plant.apply_wrench(np.zeros(6))
        plant.step()

    state = plant.get_state()
    plant.close()

    expected_z = 0.5 * gravity[2] * (t_total**2)
    expected_vz = gravity[2] * t_total

    assert np.isclose(state["H"][2, 3], expected_z, atol=2e-3)
    assert np.isclose(state["V"][5], expected_vz, atol=2e-2)


def test_compound_pi_matches_formula():
    bare_pi = np.array([
        3.646, 0.0, 0.0, -0.00835, 0.04092, 0.04017, 0.06921,
        5.656e-5, 1.313e-5, -6.494e-5
    ])
    payload = {
        "mass": 0.75,
        "dimensions": np.array([0.12, 0.12, 0.08]),
        "center": np.array([0.15, 0.0, -0.10]),
    }

    loaded_pi = compound_pi(bare_pi, payload)
    assert np.isclose(loaded_pi[0], bare_pi[0] + payload["mass"])
    assert np.allclose(loaded_pi[1:4], bare_pi[1:4] + payload["mass"] * payload["center"])
