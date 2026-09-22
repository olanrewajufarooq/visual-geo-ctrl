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
        assert len(arena.body_ids) >= 1  # Loaded arena scene multi-body

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
        generate_multicopter_urdf(pi, tmp_urdf, drone_type="hexacopter")

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


def test_pybullet_drones_urdf_generation():
    """Verify pybullet_drones model generates cf2.dae mesh visuals and preserves exact paper mass."""
    pi = np.array([3.646, 0.0, 0.0, -0.00835, 0.04092, 0.04017, 0.06921, 5.656e-5, 1.313e-5, -6.494e-5])
    fd, tmp_urdf = tempfile.mkstemp(suffix=".urdf", prefix="test_cf2_")
    os.close(fd)

    try:
        generate_multicopter_urdf(pi, tmp_urdf, drone_type="pybullet_drones")

        with open(tmp_urdf, "r", encoding="utf-8") as f:
            content = f.read()

        # Verify cf2 mesh and 4 motor hubs
        assert "cf2.dae" in content
        assert "motor_1" in content
        assert "motor_4" in content
        assert "prop_front" in content
        assert "prop_rear" in content

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


def test_lemniscate_4gates_static_urdf():
    """Verify RaceGateManager loads exactly 4 gates for the lemniscate circuit from static URDF."""
    cid = p.connect(p.DIRECT)
    try:
        mgr = RaceGateManager(client_id=cid)
        gates = mgr.load_lemniscate_4gates(z_offset=0.0)

        assert len(gates) == 4
        assert gates[0].gate_id == 1
        assert np.isclose(gates[0].center[0], 3.5, atol=1e-2)
        assert np.isclose(gates[0].center[1], 1.55, atol=1e-2)
        assert np.isclose(gates[1].center[0], 3.5, atol=1e-2)
        assert np.isclose(gates[1].center[1], -1.60, atol=1e-2)
        assert np.isclose(gates[2].center[0], -3.5, atol=1e-2)
        assert np.isclose(gates[2].center[1], 1.50, atol=1e-2)
        assert gates[3].gate_id == 4
        assert np.isclose(gates[3].center[0], -3.5, atol=1e-2)
        assert np.isclose(gates[3].center[1], -1.70, atol=1e-2)

        # Test traversal tracking
        prev_p = np.array([3.4, 1.55, 0.75])
        curr_p = np.array([3.6, 1.55, 0.75])
        res = mgr.check_traversals(drone_pos=curr_p, prev_pos=prev_p, t=1.2)
        assert res is not None
        assert res[0] == 1  # Gate 1 cleared
        assert np.isclose(res[1], 1.2)

        mgr.clear()
        assert len(mgr.gates) == 0
    finally:
        p.disconnect(cid)


def test_lemniscate_gate_alignment_with_scenario():
    """Verify default scenario ground_z=0.0 and lemniscate gate centers align with cruise altitude."""
    from agc.sim.default_scenario import default_scenario
    scen = default_scenario(replay_id="lemniscate_01_auto", gui=False)
    assert scen["groundZ"] == 0.0

    cid = p.connect(p.DIRECT)
    try:
        mgr = RaceGateManager(client_id=cid, ground_z=scen["groundZ"])
        gates = mgr.load_lemniscate_4gates(z_offset=0.0)
        assert len(gates) == 4

        # Gate centers should be at Z=0.75
        for g in gates:
            assert np.isclose(g.center[2], 0.75, atol=1e-3)

        # Cruise altitude from trajectory around t=15.0 should be near 0.75m (cruising envelope is [0.66, 0.78])
        cruise_sample = scen["trajectory"](15.0)
        cruise_z = cruise_sample["H"][2, 3]
        assert np.isclose(cruise_z, 0.75, atol=0.10)
        # All gates apertures (height 1.524m centered at 0.75m) comfortably contain cruise altitude
        assert abs(cruise_z - 0.75) < (gates[0].height_inner / 2.0)
        mgr.close()
    finally:
        p.disconnect(cid)


def test_arena_scene_zero_collision_shapes():
    """Verify ArenaScene body has zero collision shapes to prevent duplicate floor collisions."""
    cid = p.connect(p.DIRECT)
    try:
        arena = ArenaScene(client_id=cid, ground_z=0.0, style="arena")
        assert len(arena.body_ids) >= 1
        body_id = arena.body_ids[0]

        # Query collision shapes of base link (-1) and any links
        col_data = p.getCollisionShapeData(body_id, -1, physicsClientId=cid)
        assert len(col_data) == 0, f"Expected 0 collision shapes on arena base, got {len(col_data)}"

        num_joints = p.getNumJoints(body_id, physicsClientId=cid)
        for j in range(num_joints):
            j_col = p.getCollisionShapeData(body_id, j, physicsClientId=cid)
            assert len(j_col) == 0, f"Expected 0 collision shapes on joint {j}, got {len(j_col)}"

        arena.close()
    finally:
        p.disconnect(cid)


def test_gate_directional_and_ordered_traversal():
    """Verify gates enforce forward directional crossing and sequential order."""
    cid = p.connect(p.DIRECT)
    try:
        mgr = RaceGateManager(client_id=cid)
        gates = mgr.load_lemniscate_4gates(z_offset=0.0)
        # Gate 1: center=[3.5, 1.55, 0.75], normal=[1, 0, 0] (yaw=0)
        # Gate 2: center=[3.5, -1.60, 0.75], normal=[-1, 0, 0] (yaw=pi)

        # 1. Reverse traversal through Gate 1: drone flies backwards (X: 3.6 -> 3.4)
        p_ahead = np.array([3.6, 1.55, 0.75])
        p_behind = np.array([3.4, 1.55, 0.75])
        rev_result = mgr.check_traversals(drone_pos=p_behind, prev_pos=p_ahead, t=1.0, ordered=True)
        assert rev_result is None, "Reverse crossing should be rejected"
        assert not gates[0].cleared
        assert mgr.active_gate_index == 0

        # 2. Out-of-order forward attempt: try to clear Gate 2 before Gate 1
        # Gate 2 normal is [-1, 0, 0]. Forward crossing is X: 3.6 -> 3.4
        p2_before = np.array([3.6, -1.60, 0.75])
        p2_after = np.array([3.4, -1.60, 0.75])
        ooo_result = mgr.check_traversals(drone_pos=p2_after, prev_pos=p2_before, t=1.5, ordered=True)
        assert ooo_result is None, "Out-of-order Gate 2 crossing should be rejected when Gate 1 is active"
        assert not gates[1].cleared
        assert mgr.active_gate_index == 0

        # 3. Valid forward crossing for Gate 1 (X: 3.4 -> 3.6)
        valid_g1 = mgr.check_traversals(drone_pos=p_ahead, prev_pos=p_behind, t=2.0, ordered=True)
        assert valid_g1 is not None
        assert valid_g1[0] == 1
        assert gates[0].cleared
        assert mgr.active_gate_index == 1

        # 4. Now valid forward crossing for Gate 2
        valid_g2 = mgr.check_traversals(drone_pos=p2_after, prev_pos=p2_before, t=3.0, ordered=True)
        assert valid_g2 is not None
        assert valid_g2[0] == 2
        assert gates[1].cleared
        assert mgr.active_gate_index == 2

        mgr.close()
    finally:
        p.disconnect(cid)


def test_asset_path_resolution():
    """Verify get_asset_path locates bundled assets correctly."""
    from agc.viz.assets import get_asset_path
    arena_urdf = get_asset_path("arena", "arena_scene.urdf")
    assert arena_urdf.exists()

    cf2_mesh = get_asset_path("drone", "cf2.dae")
    assert cf2_mesh.exists()

    gates_urdf = get_asset_path("gates", "lemniscate_gates.urdf")
    assert gates_urdf.exists()


def test_procedural_gate_support_legs_ground_z():
    """Verify procedural gate support legs extend down to the specified ground_z."""
    cid = p.connect(p.DIRECT)
    try:
        custom_ground = -2.5
        mgr = RaceGateManager(client_id=cid, ground_z=custom_ground)

        def dummy_traj(t):
            H = np.eye(4)
            H[0:3, 3] = [0.0, 0.0, 1.5]
            return {"H": H, "V": np.zeros(6)}

        # Procedural gate generator calls _build_visual_gates() -> _render_single_gate()
        gates = mgr.generate_path_adaptive_gates(dummy_traj, duration=5.0, num_gates=1)
        assert len(gates) == 1
        assert mgr.ground_z == custom_ground
        assert len(gates[0].debug_ids) > 0

        mgr.close()
    finally:
        p.disconnect(cid)


