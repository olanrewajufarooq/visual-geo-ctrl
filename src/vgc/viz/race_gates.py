"""Authentic 3D Drone Racing Gate Passages and Track Management for PyBullet.

Digital twin of the 7-gate racing circuit from:
- Bosello et al., "Race Against the Machine: A Fully-Annotated, Open-Design Dataset
  of Autonomous and Piloted High-Speed Flight", IEEE RA-L 2024.
- Bosello et al., "On Your Own: Pro-Level Autonomous Drone Racing in Uninstrumented Arenas",
  IEEE RA-L 2026.
"""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Callable, Any
import numpy as np
import pybullet as p

from .assets import get_asset_path


@dataclass
class RaceGate:
    """Represents a single 3D drone racing gate passage."""
    gate_id: int
    center: np.ndarray  # [x, y, z] center of inner aperture
    yaw: float  # Yaw orientation in radians
    rotation: np.ndarray  # 3x3 rotation matrix
    normal: np.ndarray  # Unit vector normal to gate plane
    markers: Dict[str, np.ndarray] = field(default_factory=dict)
    width_inner: float = 1.524   # 5.0 ft standard aperture
    height_inner: float = 1.524  # 5.0 ft standard aperture
    width_outer: float = 2.134   # 7.0 ft exterior frame
    height_outer: float = 2.134  # 7.0 ft exterior frame
    cleared: bool = False
    clearance_time: Optional[float] = None
    body_ids: List[int] = field(default_factory=list)
    debug_ids: List[int] = field(default_factory=list)


# Exact 7-gate ground truth coordinates from flight-13a-trackRATM_cam_ts_sync.csv
RATM_GATES_DATA = [
    {
        "id": 1,
        "name": "Gate 1 (Mid-Field High)",
        "pos": [0.76685, 0.68646, 3.63564],
        "rpy": [-9.18e-05, 0.00086551, 0.00268851],
        "markers": {
            "m1": [0.78742, -0.06556, 2.86984],
            "m2": [0.80195, -0.07431, 4.39414],
            "m3": [0.74340, 1.43993, 2.87503],
            "m4": [0.73343, 1.44592, 4.40360],
        },
    },
    {
        "id": 2,
        "name": "Gate 2 (East Entry)",
        "pos": [9.61719, 1.73729, 1.16587],
        "rpy": [0.00014102, 0.01959254, 0.03332741],
        "markers": {
            "m1": [9.59178, 0.98003, 0.40013],
            "m2": [9.61806, 2.49343, 0.40230],
            "m3": [9.60535, 0.97314, 1.93299],
            "m4": [9.65353, 2.50253, 1.92813],
        },
    },
    {
        "id": 3,
        "name": "Gate 3 (East Exit)",
        "pos": [8.85910, -0.58629, 1.47953],
        "rpy": [0.00167761, -0.01836034, -0.02232311],
        "markers": {
            "m1": [8.88551, 0.17050, 0.71178],
            "m2": [8.84077, -1.35422, 2.24833],
            "m3": [8.85108, -1.33999, 0.71057],
            "m4": [8.85899, 0.17861, 2.24739],
        },
    },
    {
        "id": 4,
        "name": "Gate 4 (Split-S Upper Entry)",
        "pos": [-8.48038, -2.28179, 4.24184],
        "rpy": [-0.00012619, -0.02510621, -0.01135005],
        "markers": {
            "m1": [-8.50794, -1.53255, 3.48028],
            "m2": [-8.47192, -3.04571, 3.47915],
            "m3": [-8.49917, -1.51074, 5.00558],
            "m4": [-8.44205, -3.03817, 5.00233],
        },
    },
    {
        "id": 5,
        "name": "Gate 5 (Split-S Lower Exit)",
        "pos": [-8.50692, -2.29258, 2.06406],
        "rpy": [0.00085347, -0.01667872, -0.01223074],
        "markers": {
            "m1": [-8.51112, -1.53210, 1.30130],
            "m2": [-8.50279, -3.07412, 2.82863],
            "m3": [-8.50594, -1.52950, 2.81958],
            "m4": [-8.50774, -3.03460, 1.30675],
        },
    },
    {
        "id": 6,
        "name": "Gate 6 (Return Straight)",
        "pos": [-4.15249, -1.86915, 1.16716],
        "rpy": [-4.54497e-05, 0.00082166, -0.00692869],
        "markers": {
            "m1": [-4.16463, -1.11509, 0.38968],
            "m2": [-4.16252, -1.09604, 1.93328],
            "m3": [-4.15023, -2.63303, 0.40452],
            "m4": [-4.13324, -2.63216, 1.94124],
        },
    },
    {
        "id": 7,
        "name": "Gate 7 (Chicane / Home Stretch)",
        "pos": [-3.77529, 2.54654, 1.48086],
        "rpy": [0.00010001, 0.00042970, -0.00155230],
        "markers": {
            "m1": [-3.75797, 1.78875, 0.70447],
            "m2": [-3.77895, 1.78285, 2.25378],
            "m3": [-3.77166, 3.30538, 0.70795],
            "m4": [-3.79272, 3.30904, 2.25708],
        },
    },
]


class RaceGateManager:
    """Manages spawning, rendering, and traversal tracking for 3D racing gates."""

    def __init__(self, client_id: int, ground_z: float = 0.0):
        self.client_id = client_id
        self.ground_z = float(ground_z)
        self.gates: List[RaceGate] = []
        self.last_cleared_gate: Optional[int] = None
        self.active_gate_index: int = 0
        self.lap_count: int = 0
        self.gate_split_times: Dict[int, float] = {}

    def load_ratm_track(self, z_offset: float = 0.0) -> List[RaceGate]:
        """Load the exact 7-gate RATM benchmark track from the original papers."""
        self.clear()
        for g_data in RATM_GATES_DATA:
            pos = np.array(g_data["pos"], dtype=float)
            pos[2] += z_offset
            yaw = float(g_data["rpy"][2])

            c = np.cos(yaw)
            s = np.sin(yaw)
            R = np.array([
                [c, -s, 0.0],
                [s, c, 0.0],
                [0.0, 0.0, 1.0],
            ], dtype=float)
            # Normal vector along gate traversal direction (local +X rotated by yaw)
            normal = R[:, 0]

            markers = {}
            for m_key, m_val in g_data["markers"].items():
                m_pos = np.array(m_val, dtype=float)
                m_pos[2] += z_offset
                markers[m_key] = m_pos

            gate = RaceGate(
                gate_id=g_data["id"],
                center=pos,
                yaw=yaw,
                rotation=R,
                normal=normal,
                markers=markers,
            )
            self.gates.append(gate)

        self._build_visual_gates()
        return self.gates

    def load_lemniscate_4gates(self, z_offset: float = 0.0) -> List[RaceGate]:
        """Load the authentic 4-gate lemniscate racing circuit from static URDF."""
        self.clear()
        urdf_path = get_asset_path("gates", "lemniscate_gates.urdf")
        body_id = None
        if urdf_path.exists():
            body_id = p.loadURDF(
                str(urdf_path).replace("\\", "/"),
                basePosition=[0.0, 0.0, z_offset],
                baseOrientation=[0.0, 0.0, 0.0, 1.0],
                useFixedBase=True,
                flags=p.URDF_MERGE_FIXED_LINKS,
                physicsClientId=self.client_id,
            )

        # Exact centers and orientations matching the URDF model and lemniscate_01_auto trajectory
        lemniscate_specs = [
            {"id": 1, "pos": np.array([3.5, 1.55, 0.75 + z_offset]), "yaw": 0.0},
            {"id": 2, "pos": np.array([3.5, -1.60, 0.75 + z_offset]), "yaw": float(np.pi)},
            {"id": 3, "pos": np.array([-3.5, 1.50, 0.75 + z_offset]), "yaw": float(np.pi)},
            {"id": 4, "pos": np.array([-3.5, -1.70, 0.75 + z_offset]), "yaw": 0.0},
        ]

        for spec in lemniscate_specs:
            yaw = spec["yaw"]
            c, s = np.cos(yaw), np.sin(yaw)
            R = np.array([
                [c, -s, 0.0],
                [s, c, 0.0],
                [0.0, 0.0, 1.0],
            ], dtype=float)
            normal = R[:, 0]

            gate = RaceGate(
                gate_id=spec["id"],
                center=spec["pos"],
                yaw=yaw,
                rotation=R,
                normal=normal,
                width_inner=1.524,
                height_inner=1.524,
            )
            if body_id is not None:
                gate.body_ids.append(body_id)
            self.gates.append(gate)

        # Fallback to procedural line visualization only if URDF file is absent
        if body_id is None:
            self._build_visual_gates()

        return self.gates

    def generate_path_gates(
        self,
        trajectory_fn: Callable[[float], Dict[str, Any]],
        duration: float,
        num_gates: int = 4,
    ) -> List[RaceGate]:
        """Sample gates along an arbitrary continuous trajectory (e.g. lemniscate, ellipse)."""
        self.clear()
        times = np.linspace(duration * 0.15, duration * 0.85, num_gates)

        for i, t in enumerate(times, start=1):
            sample = trajectory_fn(t)
            H = sample["H"]
            center = np.copy(H[0:3, 3])
            # Desired heading from tangent velocity or rotation matrix
            R = np.copy(H[0:3, 0:3])
            normal = R[:, 0]
            yaw = float(np.arctan2(normal[1], normal[0]))

            c = np.cos(yaw)
            s = np.sin(yaw)
            R_yaw = np.array([
                [c, -s, 0.0],
                [s, c, 0.0],
                [0.0, 0.0, 1.0],
            ], dtype=float)

            gate = RaceGate(
                gate_id=i,
                center=center,
                yaw=yaw,
                rotation=R_yaw,
                normal=R_yaw[:, 0],
            )
            self.gates.append(gate)

        self._build_visual_gates()
        return self.gates

    def _build_visual_gates(self):
        """Construct realistic 3D racing gates with banners, corner markers, and labels."""
        for gate in self.gates:
            self._render_single_gate(gate)

    def _render_single_gate(self, gate: RaceGate):
        """Render a single gate's frame, fabric banner, corner markers, and ID badge."""
        c = gate.center
        R = gate.rotation
        w_in = gate.width_inner
        h_in = gate.height_inner
        post_r = 0.035
        border = 0.25

        # Colors: High-contrast racing banner (Electric Blue & Neon Orange)
        banner_blue = [0.05, 0.45, 0.95, 0.95]
        banner_orange = [1.0, 0.35, 0.05, 0.95]
        frame_color = [0.68, 0.70, 0.74, 1.0]  # Light aluminium frame (high contrast against dark floor)

        # Alternating gate colors for visual distinction
        primary_color = banner_orange if gate.gate_id % 2 == 1 else banner_blue

        # Inner aperture boundaries in local frame [x_b, y_b, z_b]
        # x_b = forward (normal to gate), y_b = lateral, z_b = vertical
        half_w = w_in / 2.0
        half_h = h_in / 2.0

        # Local corner positions of inner aperture
        corners_local = [
            np.array([0.0, -half_w, -half_h]),  # bottom-left
            np.array([0.0, half_w, -half_h]),   # bottom-right
            np.array([0.0, half_w, half_h]),    # top-right
            np.array([0.0, -half_w, half_h]),   # top-left
        ]
        corners_world = [c + R @ cl for cl in corners_local]

        # 1. Outer Frame / Uprights & Crossbar (Debug lines with high width)
        # Left upright
        p_bl = corners_world[0]
        p_br = corners_world[1]
        p_tr = corners_world[2]
        p_tl = corners_world[3]

        # Inner aperture border
        self._add_line(gate, p_bl, p_br, primary_color[:3], width=4.0)
        self._add_line(gate, p_br, p_tr, primary_color[:3], width=4.0)
        self._add_line(gate, p_tr, p_tl, primary_color[:3], width=4.0)
        self._add_line(gate, p_tl, p_bl, primary_color[:3], width=4.0)

        # Outer banner frame (outer chevrons)
        outer_corners_local = [
            np.array([0.0, -(half_w + border), -(half_h + border)]),
            np.array([0.0, (half_w + border), -(half_h + border)]),
            np.array([0.0, (half_w + border), (half_h + border)]),
            np.array([0.0, (half_w + border), (half_h + border)]),
        ]
        outer_corners_world = [c + R @ cl for cl in outer_corners_local]

        # Outer border
        self._add_line(gate, outer_corners_world[0], outer_corners_world[1], frame_color[:3], width=2.5)
        self._add_line(gate, outer_corners_world[1], outer_corners_world[2], frame_color[:3], width=2.5)
        self._add_line(gate, outer_corners_world[2], outer_corners_world[3], frame_color[:3], width=2.5)
        self._add_line(gate, outer_corners_world[3], outer_corners_world[0], frame_color[:3], width=2.5)

        # Diagonals on banner corners
        for i in range(4):
            self._add_line(gate, corners_world[i], outer_corners_world[i], primary_color[:3], width=3.0)

        # 2. Ground Support Legs extending down to floor with ground threshold stabilizer
        z_floor = self.ground_z
        leg_l_bottom = np.array([corners_world[0][0], corners_world[0][1], z_floor])
        leg_r_bottom = np.array([corners_world[1][0], corners_world[1][1], z_floor])
        self._add_line(gate, corners_world[0], leg_l_bottom, frame_color[:3], width=4.0)
        self._add_line(gate, corners_world[1], leg_r_bottom, frame_color[:3], width=4.0)
        self._add_line(gate, leg_l_bottom, leg_r_bottom, frame_color[:3], width=4.0)

        # 3. Retroreflective MoCap Corner Markers (White/Silver spheres at 4 corners)
        marker_positions = list(gate.markers.values()) if gate.markers else corners_world
        for m_pos in marker_positions:
            self._add_marker_sphere(gate, m_pos)

    def _add_line(self, gate: RaceGate, p1: np.ndarray, p2: np.ndarray, color: List[float], width: float = 2.0):
        """Add a persistent debug line and track its ID with the gate."""
        line_id = p.addUserDebugLine(
            p1.tolist(),
            p2.tolist(),
            lineColorRGB=color,
            lineWidth=width,
            lifeTime=0,
            physicsClientId=self.client_id,
        )
        gate.debug_ids.append(line_id)

    def _add_marker_sphere(self, gate: RaceGate, pos: np.ndarray):
        """Render a small retroreflective MoCap corner marker cross."""
        d = 0.035
        c = [0.95, 0.95, 0.95]
        self._add_line(gate, pos - np.array([d, 0, 0]), pos + np.array([d, 0, 0]), c, width=3.0)
        self._add_line(gate, pos - np.array([0, d, 0]), pos + np.array([0, d, 0]), c, width=3.0)
        self._add_line(gate, pos - np.array([0, 0, d]), pos + np.array([0, 0, d]), c, width=3.0)

    def check_traversals(
        self,
        drone_pos: np.ndarray,
        prev_pos: Optional[np.ndarray] = None,
        t: float = 0.0,
        ordered: bool = True,
    ) -> Optional[Tuple[int, float]]:
        """Check whether the drone has crossed through the active gate aperture (directional & ordered)."""
        if prev_pos is None or not self.gates:
            return None

        # Determine which gate(s) to check
        if ordered:
            gates_to_check = [self.gates[self.active_gate_index]]
        else:
            gates_to_check = [g for g in self.gates if not g.cleared]

        for gate in gates_to_check:
            # Vector from gate center to drone positions
            d_curr = drone_pos - gate.center
            d_prev = prev_pos - gate.center

            # Projection along gate normal (traversal axis)
            s_curr = np.dot(d_curr, gate.normal)
            s_prev = np.dot(d_prev, gate.normal)

            # Directional traversal check: must cross forward from entry (s <= 0) to exit (s > 0)
            if s_prev <= 0.0 and s_curr > 0.0:
                # Interpolate exact in-plane crossing position
                denom = abs(s_prev) + abs(s_curr)
                alpha = abs(s_prev) / (denom if denom > 1e-9 else 1.0)
                cross_pt = prev_pos + alpha * (drone_pos - prev_pos)
                cross_rel = cross_pt - gate.center

                # Project into local lateral (y_b) and vertical (z_b) coordinates
                y_b = gate.rotation[:, 1]
                z_b = gate.rotation[:, 2]
                lat_dist = abs(np.dot(cross_rel, y_b))
                vert_dist = abs(np.dot(cross_rel, z_b))

                # If within inner aperture (with generous 20% margin for racing clearance)
                if lat_dist <= (gate.width_inner / 2.0) * 1.2 and vert_dist <= (gate.height_inner / 2.0) * 1.2:
                    gate.cleared = True
                    gate.clearance_time = t
                    self.last_cleared_gate = gate.gate_id
                    self.gate_split_times[gate.gate_id] = t
                    self._flash_gate_cleared(gate)

                    if ordered:
                        self.active_gate_index = (self.active_gate_index + 1) % len(self.gates)
                        if self.active_gate_index == 0:
                            self.lap_count += 1
                            for g in self.gates:
                                g.cleared = False
                    else:
                        # Multi-lap support: reset cleared flags when all gates are cleared
                        if all(g.cleared for g in self.gates):
                            self.lap_count += 1
                            for g in self.gates:
                                g.cleared = False

                    return gate.gate_id, t

        return None

    def _flash_gate_cleared(self, gate: RaceGate):
        """Visual pulse when a gate is successfully cleared (bright neon green)."""
        c = gate.center
        R = gate.rotation
        half_w = gate.width_inner / 2.0
        half_h = gate.height_inner / 2.0

        corners_local = [
            np.array([0.0, -half_w, -half_h]),
            np.array([0.0, half_w, -half_h]),
            np.array([0.0, half_w, half_h]),
            np.array([0.0, -half_w, half_h]),
        ]
        corners_world = [c + R @ cl for cl in corners_local]

        # Inner bright neon green frame with temporary lifetime
        for i in range(4):
            p.addUserDebugLine(
                corners_world[i].tolist(),
                corners_world[(i + 1) % 4].tolist(),
                lineColorRGB=[0.1, 1.0, 0.2],
                lineWidth=5.0,
                lifeTime=1.5,
                physicsClientId=self.client_id,
            )

    def clear(self):
        """Remove all gates and visual items from PyBullet."""
        removed_bodies = set()
        for gate in self.gates:
            for d_id in gate.debug_ids:
                try:
                    p.removeUserDebugItem(d_id, physicsClientId=self.client_id)
                except Exception:
                    pass
            for b_id in gate.body_ids:
                if b_id not in removed_bodies:
                    try:
                        p.removeBody(b_id, physicsClientId=self.client_id)
                    except Exception:
                        pass
                    removed_bodies.add(b_id)
        self.gates.clear()
        self.last_cleared_gate = None
        self.active_gate_index = 0
        self.lap_count = 0
        self.gate_split_times.clear()

    def close(self):
        """Alias for clear."""
        self.clear()
