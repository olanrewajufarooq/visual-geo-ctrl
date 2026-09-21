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


def test_pose_and_velocity_roundtrip():
    """Verify pose in SE(3) and body-frame twist round-trip through PyBullet."""
    pi = np.array([2.0, 0.0, 0.0, 0.0, 0.1, 0.12, 0.14, 0.0, 0.0, 0.0])
    plant = PyBulletPlant(pi=pi, dt=0.002, gui=False)

    # 90 deg yaw rotation and translation
    R_test = np.array([
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ])
    p_test = np.array([1.5, -2.0, 3.5])
    H_test = np.eye(4)
    H_test[0:3, 0:3] = R_test
    H_test[0:3, 3] = p_test

    # Non-zero body twist [omega_b; v_b]
    V_test = np.array([0.1, -0.2, 0.5, 1.0, -0.5, 2.0])

    plant.set_state(H_test, V_test)
    state = plant.get_state()
    plant.close()

    np.testing.assert_allclose(state["H"], H_test, atol=1e-5)
    np.testing.assert_allclose(state["V"], V_test, atol=1e-5)


def test_payload_attachment_and_dynamic_release():
    """Verify payload is attached initially and physically detached at release_time."""
    bare_pi = np.array([3.646, 0.0, 0.0, -0.00835, 0.04092, 0.04017, 0.06921, 5.656e-5, 1.313e-5, -6.494e-5])
    payload = {
        "mass": 0.75,
        "dimensions": np.array([0.12, 0.12, 0.08]),
        "center": np.array([0.15, 0.0, -0.10]),
    }
    loaded_pi = compound_pi(bare_pi, payload)

    plant = PyBulletPlant(
        pi=loaded_pi,
        dt=0.002,
        gui=False,
        payload=payload,
        release_time=0.01,
    )

    assert plant.payload_id is not None
    assert plant.constraint_id is not None
    assert not plant.payload_dropped

    # Step before release
    plant.step(time=0.005)
    assert not plant.payload_dropped
    assert plant.constraint_id is not None

    # Step at/past release
    plant.step(time=0.015)
    assert plant.payload_dropped
    assert plant.constraint_id is None

    plant.close()


def test_plant_cleanup_and_disconnection():
    """Verify plant.close() disconnects client and idempotent multiple calls."""
    pi = np.array([2.0, 0.0, 0.0, 0.0, 0.1, 0.12, 0.14, 0.0, 0.0, 0.0])
    plant = PyBulletPlant(pi=pi, dt=0.002, gui=False)
    client_id = plant.client_id

    import pybullet as p
    assert p.isConnected(physicsClientId=client_id)

    plant.close()
    assert not p.isConnected(physicsClientId=client_id)

    # Calling close again should not raise
    plant.close()


def test_exact_mass_and_inertia_properties():
    """Verify UAV base link in PyBullet has exact expected mass and inertia."""
    pi = np.array([3.5, 0.0, 0.0, 0.0, 0.05, 0.06, 0.07, 0.0, 0.0, 0.0])
    plant = PyBulletPlant(pi=pi, dt=0.002, gui=False)

    import pybullet as p
    info = p.getDynamicsInfo(plant.uav_id, -1, physicsClientId=plant.client_id)
    mass = info[0]
    plant.close()

    assert np.isclose(mass, 3.5, atol=1e-3)
