"""Automated unit tests for realistic ArenaScene, RaceGateManager, FpvOsd, and Hexacopter URDF."""

import tempfile
import os
import numpy as np
import pybullet as p
import pytest

from agc.viz.arena_scene import ArenaScene
from agc.viz.race_gates import RaceGateManager, RaceGate, RATM_GATES_DATA
from agc.viz.fpv_osd import FpvOsd
from agc.plant.drone_urdf import generate_multicopter_urdf


def test_arena_scene_creation_and_cleanup():
    """Verify ArenaScene builds floor, launch pad, safety perimeter and cleans up."""
    cid = p.connect(p.DIRECT)
    try:
        arena = ArenaScene(client_id=cid, ground_z=-1.5, style="arena")
        assert len(arena.body_ids) >= 2  # Apron and racing floor
        assert len(arena.debug_item_ids) > 0

        # Test cleanup
        arena.close()
        assert len(arena.body_ids) == 0
        assert len(arena.debug_item_ids) == 0
    finally:
        p.disconnect(cid)


def test_ratm_gate_benchmark_geometry():
    """Verify the 7-gate RATM benchmark track loaded matches the original papers."""
    cid = p.connect(p.DIRECT)
    try:
        mgr = RaceGateManager(client_id=cid)
        gates = mgr.load_ratm_track()

        assert len(gates) == 7

        # Verify Gate 1 high mid-field passage
        g1 = gates[0]
        assert g1.gate_id == 1
        assert np.isclose(g1.center[0], 0.76685, atol=1e-3)
        assert np.isclose(g1.center[2], 3.63564, atol=1e-3)
        assert np.isclose(g1.width_inner, 1.524, atol=1e-3)

        # Verify Split-S Gate pair (Gate 4 high entry, Gate 5 lower exit)
        g4 = gates[3]
        g5 = gates[4]
        assert g4.gate_id == 4
        assert g5.gate_id == 5
        # Same lateral coordinates in the arena
        assert np.isclose(g4.center[0], g5.center[0], atol=0.05)
        assert np.isclose(g4.center[1], g5.center[1], atol=0.05)
        # Vertical separation forming the Split-S dive:
        assert g4.center[2] > 4.0   # Upper entry (~4.24 m)
        assert g5.center[2] < 2.5   # Lower exit (~2.06 m)
        assert g4.center[2] - g5.center[2] > 2.0

        # Verify all 4 MoCap markers present and check aperture dimension
        for gate in gates:
            assert len(gate.markers) == 4
            m1 = gate.markers["m1"]
            m2 = gate.markers["m2"]
            # Marker span across adjacent edge (~1.52m) or diagonal (~2.15m)
            marker_dist = float(np.linalg.norm(m2 - m1))
            assert marker_dist > 1.4 and marker_dist < 2.30

        mgr.close()
        assert len(mgr.gates) == 0
    finally:
        p.disconnect(cid)


def test_path_adaptive_gates():
    """Verify dynamic gate generation along an arbitrary continuous trajectory."""
    cid = p.connect(p.DIRECT)
    try:
        mgr = RaceGateManager(client_id=cid)

        def circle_traj(t):
            theta = 0.5 * t
            H = np.eye(4)
            H[0:3, 3] = [3.0 * np.cos(theta), 3.0 * np.sin(theta), 1.5]
            # Velocity tangent
            tangent = np.array([-np.sin(theta), np.cos(theta), 0.0])
            norm_t = tangent / np.linalg.norm(tangent)
            H[0:3, 0] = norm_t
            return {"H": H, "V": np.zeros(6)}

        gates = mgr.generate_path_adaptive_gates(circle_traj, duration=10.0, num_gates=4)
        assert len(gates) == 4
        for i, gate in enumerate(gates, start=1):
            assert gate.gate_id == i
            assert np.isclose(gate.center[2], 1.5, atol=1e-3)
            # Distance from origin should be 3.0 m
            rad = np.linalg.norm(gate.center[:2])
            assert np.isclose(rad, 3.0, atol=1e-2)

        mgr.close()
    finally:
        p.disconnect(cid)


def test_gate_traversal_detection():
    """Verify that crossing through the inner aperture triggers clearance and records split time."""
    cid = p.connect(p.DIRECT)
    try:
        mgr = RaceGateManager(client_id=cid)
        mgr.load_ratm_track()

        g1 = mgr.gates[0]  # Center at [0.767, 0.686, 3.636], normal along +X
        assert not g1.cleared

        # Pre-crossing position (X < 0.767)
        p_prev = np.array([0.0, 0.686, 3.636])
        # Post-crossing position (X > 0.767)
        p_curr = np.array([1.5, 0.686, 3.636])

        traversal = mgr.check_traversals(drone_pos=p_curr, prev_pos=p_prev, t=4.52)
        assert traversal is not None
        gate_id, split_t = traversal
        assert gate_id == 1
        assert np.isclose(split_t, 4.52)
        assert g1.cleared
        assert mgr.last_cleared_gate == 1

        # Second check should not re-trigger
        p_next = np.array([2.0, 0.686, 3.636])
        assert mgr.check_traversals(drone_pos=p_next, prev_pos=p_curr, t=5.0) is None

        mgr.close()
    finally:
        p.disconnect(cid)


def test_fpv_osd_state_and_toggle():
    """Verify FpvOsd updates telemetry, handles toggling, and clears without errors."""
    cid = p.connect(p.DIRECT)
    try:
        osd = FpvOsd(client_id=cid, enabled=True)
        assert osd.enabled

        pos = np.array([1.0, 2.0, 1.5])
        R = np.eye(3)
        vel = np.array([5.2, 0.1, -0.2])

        osd.update(
            t=10.0,
            pos=pos,
            R=R,
            vel=vel,
            pos_err=0.045,
            s_norm=0.12,
            est_m=3.65,
            true_m=3.65,
            payload_dropped=False,
            mode="BREGMAN",
            coriolis="C1",
        )
        assert osd.hud_text_id is not None

        # Test gate notification
        osd.trigger_gate_cleared(gate_id=3, split_time=8.5, total_gates=7)
        osd.update(
            t=9.0,
            pos=pos,
            R=R,
            vel=vel,
            pos_err=0.045,
            s_norm=0.12,
            est_m=3.65,
            true_m=3.65,
            payload_dropped=False,
        )
        assert osd.gate_banner_id is not None

        # Test toggle
        osd.toggle()
        assert not osd.enabled
        osd.update(
            t=10.0, pos=pos, R=R, vel=vel, pos_err=0.0, s_norm=0.0, est_m=3.0, true_m=3.0, payload_dropped=False
        )
        assert osd.hud_text_id is None

        osd.close()
    finally:
        p.disconnect(cid)


def test_high_fidelity_hexacopter_urdf_generation():
    """Verify high-detail URDF preserves exact physical mass and CoM while adding visual elements."""
    pi = np.array([3.646, 0.0, 0.0, -0.00835, 0.04092, 0.04017, 0.06921, 5.656e-5, 1.313e-5, -6.494e-5])
    fd, tmp_urdf = tempfile.mkstemp(suffix=".urdf", prefix="test_hex_")
    os.close(fd)

    try:
        generate_multicopter_urdf(pi, tmp_urdf)

        with open(tmp_urdf, "r", encoding="utf-8") as f:
            content = f.read()

        # Check visual components are generated
        assert "fpv_case" in content
        assert "camera_lens" in content
        assert "gps_mast" in content
        assert "led_green" in content
        assert "led_red" in content
        assert "motor_stator" in content

        cid = p.connect(p.DIRECT)
        try:
            uav_id = p.loadURDF(tmp_urdf, [0, 0, 0], [0, 0, 0, 1], flags=p.URDF_MERGE_FIXED_LINKS, physicsClientId=cid)
            dyn_info = p.getDynamicsInfo(uav_id, -1, physicsClientId=cid)
            mass = dyn_info[0]
            # Exact mass invariance check
            assert np.isclose(mass, 3.646, atol=1e-4)
        finally:
            p.disconnect(cid)

    finally:
        if os.path.exists(tmp_urdf):
            os.remove(tmp_urdf)
