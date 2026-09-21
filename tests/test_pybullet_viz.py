"""Tests for PyBulletVisualizer module."""

import numpy as np
import pybullet as p
from agc.plant.pybullet_plant import PyBulletPlant
from agc.viz.pybullet_viz import PyBulletVisualizer


def test_visualizer_initialization_and_methods():
    pi = np.array([2.0, 0.0, 0.0, 0.0, 0.1, 0.12, 0.14, 0.0, 0.0, 0.0])
    # In DIRECT mode for automated headless testing
    cid = p.connect(p.DIRECT)
    uav_id = p.createMultiBody(baseMass=2.0, physicsClientId=cid)

    viz = PyBulletVisualizer(
        client_id=cid,
        uav_id=uav_id,
        ground_z=-1.5,
        sim_speed=2.0,
        enable_pacing=False,
    )

    # Test draw_reference_path with a dummy trajectory function
    def dummy_traj(t):
        H = np.eye(4)
        H[0:3, 3] = [np.cos(t), np.sin(t), 1.0]
        return {"H": H, "V": np.zeros(6)}

    viz.draw_reference_path(dummy_traj, duration=1.0, dt_sample=0.2)

    # Test update call
    for step in range(10):
        viz.update(
            t=step * 0.02,
            pos_err=0.01,
            s_norm=0.05,
            est_m=2.0,
            true_m=2.0,
            payload_dropped=False,
            step_idx=step,
        )

    p.disconnect(cid)


def test_plant_with_gui_flag_in_direct_mode():
    pi = np.array([2.0, 0.0, 0.0, 0.0, 0.1, 0.12, 0.14, 0.0, 0.0, 0.0])
    plant = PyBulletPlant(pi=pi, dt=0.002, gui=False, sim_speed=2.0, enable_pacing=False)
    assert plant.uav_id >= 0
    state = plant.get_state()
    assert state["H"].shape == (4, 4)
    plant.close()
