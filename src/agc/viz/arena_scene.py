"""Realistic indoor flight arena environment and ground styling for PyBullet.

Digital twin of the TII indoor flight arena (25m x 9.7m x 7m) used in:
- Bosello et al., "Race Against the Machine", IEEE RA-L 2024.
- Bosello et al., "On Your Own", IEEE RA-L 2026.
"""

from typing import List, Optional
import numpy as np
import pybullet as p


class ArenaScene:
    """Manages the visual flight arena floor, launch pad, safety boundaries, and netting posts."""

    def __init__(
        self,
        client_id: int,
        ground_z: float = 0.0,
        length: float = 25.0,
        width: float = 10.0,
        height: float = 6.0,
        style: str = "arena",
    ):
        self.client_id = client_id
        self.ground_z = float(ground_z)
        self.length = float(length)
        self.width = float(width)
        self.height = float(height)
        self.style = str(style).lower()

        self.body_ids: List[int] = []
        self.debug_item_ids: List[int] = []

        self._build()

    def _build(self):
        """Construct arena ground, markings, and enclosure."""
        half_l = self.length / 2.0
        half_w = self.width / 2.0

        if self.style == "arena":
            self._build_arena_floor(half_l, half_w)
            self._build_launch_pad(x=0.0, y=0.0)
            self._build_safety_perimeter(half_l, half_w)
            self._build_truss_posts(half_l, half_w)
        elif self.style == "grid":
            self._build_coordinate_grid(size=max(half_l, half_w), step=1.0)
        # style "plane" leaves default plane only

    def _build_arena_floor(self, half_l: float, half_w: float):
        """Create dark industrial epoxy arena floor with an outer runoff apron."""
        # 1. Outer runoff apron (dark slate)
        apron_l = half_l + 3.0
        apron_w = half_w + 3.0
        vis_apron = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[apron_l, apron_w, 0.02],
            rgbaColor=[0.08, 0.09, 0.10, 1.0],
            physicsClientId=self.client_id,
        )
        col_floor = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[apron_l, apron_w, 0.02],
            physicsClientId=self.client_id,
        )
        apron_id = p.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=col_floor,
            baseVisualShapeIndex=vis_apron,
            basePosition=[0.0, 0.0, self.ground_z - 0.02],
            physicsClientId=self.client_id,
        )
        self.body_ids.append(apron_id)

        # 2. Main racing flight floor (smooth charcoal epoxy)
        vis_floor = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[half_l, half_w, 0.005],
            rgbaColor=[0.14, 0.15, 0.17, 1.0],
            physicsClientId=self.client_id,
        )
        floor_id = p.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=-1,
            baseVisualShapeIndex=vis_floor,
            basePosition=[0.0, 0.0, self.ground_z - 0.005],
            physicsClientId=self.client_id,
        )
        self.body_ids.append(floor_id)

        # 3. Longitudinal center flight corridor line (dashed white guidance line)
        dash_len = 1.0
        gap_len = 0.5
        x_cur = -half_l + 0.5
        while x_cur < half_l:
            x_end = min(x_cur + dash_len, half_l)
            line_id = p.addUserDebugLine(
                [x_cur, 0.0, self.ground_z + 0.002],
                [x_end, 0.0, self.ground_z + 0.002],
                lineColorRGB=[0.85, 0.88, 0.92],
                lineWidth=2.0,
                lifeTime=0,
                physicsClientId=self.client_id,
            )
            self.debug_item_ids.append(line_id)
            x_cur += dash_len + gap_len

        # 4. Arena coordinate distance grid ticks along X and Y
        for x in np.arange(-half_l + 2.0, half_l, 2.0):
            line_id = p.addUserDebugLine(
                [float(x), -half_w, self.ground_z + 0.001],
                [float(x), half_w, self.ground_z + 0.001],
                lineColorRGB=[0.22, 0.24, 0.27],
                lineWidth=1.0,
                lifeTime=0,
                physicsClientId=self.client_id,
            )
            self.debug_item_ids.append(line_id)

        for y in np.arange(-half_w + 1.0, half_w, 1.0):
            line_id = p.addUserDebugLine(
                [-half_l, float(y), self.ground_z + 0.001],
                [half_l, float(y), self.ground_z + 0.001],
                lineColorRGB=[0.22, 0.24, 0.27],
                lineWidth=1.0,
                lifeTime=0,
                physicsClientId=self.client_id,
            )
            self.debug_item_ids.append(line_id)

    def _build_launch_pad(self, x: float = 0.0, y: float = 0.0):
        """Construct realistic circular launch pad / helipad with cardinal markings."""
        # Concentric rings
        z_pad = self.ground_z + 0.003
        r_outer = 0.85
        r_inner = 0.50
        n_seg = 36
        angles = np.linspace(0, 2 * np.pi, n_seg + 1)

        # Outer cyan circle
        for i in range(n_seg):
            p1 = [x + r_outer * np.cos(angles[i]), y + r_outer * np.sin(angles[i]), z_pad]
            p2 = [x + r_outer * np.cos(angles[i + 1]), y + r_outer * np.sin(angles[i + 1]), z_pad]
            self.debug_item_ids.append(
                p.addUserDebugLine(p1, p2, [0.0, 0.85, 1.0], lineWidth=2.5, lifeTime=0, physicsClientId=self.client_id)
            )

        # Inner yellow circle
        for i in range(n_seg):
            p1 = [x + r_inner * np.cos(angles[i]), y + r_inner * np.sin(angles[i]), z_pad]
            p2 = [x + r_inner * np.cos(angles[i + 1]), y + r_inner * np.sin(angles[i + 1]), z_pad]
            self.debug_item_ids.append(
                p.addUserDebugLine(p1, p2, [1.0, 0.82, 0.1], lineWidth=2.0, lifeTime=0, physicsClientId=self.client_id)
            )

        # Central "H" landing mark
        h_h = 0.25
        h_w = 0.18
        # Left bar
        self.debug_item_ids.append(
            p.addUserDebugLine([x - h_w, y - h_h, z_pad], [x - h_w, y + h_h, z_pad], [1.0, 1.0, 1.0], lineWidth=3.0, lifeTime=0, physicsClientId=self.client_id)
        )
        # Right bar
        self.debug_item_ids.append(
            p.addUserDebugLine([x + h_w, y - h_h, z_pad], [x + h_w, y + h_h, z_pad], [1.0, 1.0, 1.0], lineWidth=3.0, lifeTime=0, physicsClientId=self.client_id)
        )
        # Crossbar
        self.debug_item_ids.append(
            p.addUserDebugLine([x - h_w, y, z_pad], [x + h_w, y, z_pad], [1.0, 1.0, 1.0], lineWidth=3.0, lifeTime=0, physicsClientId=self.client_id)
        )

        # Forward orientation arrow (+X heading)
        arrow_tip = [x + r_outer + 0.35, y, z_pad]
        arrow_base = [x + r_outer + 0.05, y, z_pad]
        self.debug_item_ids.append(
            p.addUserDebugLine(arrow_base, arrow_tip, [0.0, 1.0, 0.4], lineWidth=3.0, lifeTime=0, physicsClientId=self.client_id)
        )
        self.debug_item_ids.append(
            p.addUserDebugLine(arrow_tip, [x + r_outer + 0.20, y + 0.10, z_pad], [0.0, 1.0, 0.4], lineWidth=3.0, lifeTime=0, physicsClientId=self.client_id)
        )
        self.debug_item_ids.append(
            p.addUserDebugLine(arrow_tip, [x + r_outer + 0.20, y - 0.10, z_pad], [0.0, 1.0, 0.4], lineWidth=3.0, lifeTime=0, physicsClientId=self.client_id)
        )

    def _build_safety_perimeter(self, half_l: float, half_w: float):
        """Draw high-contrast yellow/black hazard perimeter around the active flight boundary."""
        corners = [
            [-half_l, -half_w],
            [half_l, -half_w],
            [half_l, half_w],
            [-half_l, half_w],
            [-half_l, -half_w],
        ]
        z_b = self.ground_z + 0.003
        for i in range(4):
            c1 = [corners[i][0], corners[i][1], z_b]
            c2 = [corners[i + 1][0], corners[i + 1][1], z_b]
            # Yellow border
            self.debug_item_ids.append(
                p.addUserDebugLine(c1, c2, [1.0, 0.78, 0.0], lineWidth=3.5, lifeTime=0, physicsClientId=self.client_id)
            )

    def _build_truss_posts(self, half_l: float, half_w: float):
        """Build 4 vertical safety enclosure truss columns and top boundary cable."""
        col_positions = [
            [-half_l, -half_w],
            [half_l, -half_w],
            [half_l, half_w],
            [-half_l, half_w],
        ]
        h = self.height
        radius = 0.06

        for cp in col_positions:
            vis_col = p.createVisualShape(
                p.GEOM_CYLINDER,
                radius=radius,
                length=h,
                rgbaColor=[0.35, 0.38, 0.44, 0.85],
                physicsClientId=self.client_id,
            )
            col_id = p.createMultiBody(
                baseMass=0.0,
                baseCollisionShapeIndex=-1,
                baseVisualShapeIndex=vis_col,
                basePosition=[cp[0], cp[1], self.ground_z + h / 2.0],
                physicsClientId=self.client_id,
            )
            self.body_ids.append(col_id)

        # Top boundary cables
        z_top = self.ground_z + h
        for i in range(4):
            p1 = [col_positions[i][0], col_positions[i][1], z_top]
            p2 = [col_positions[(i + 1) % 4][0], col_positions[(i + 1) % 4][1], z_top]
            self.debug_item_ids.append(
                p.addUserDebugLine(p1, p2, [0.3, 0.45, 0.6], lineWidth=1.5, lifeTime=0, physicsClientId=self.client_id)
            )

    def _build_coordinate_grid(self, size: float = 8.0, step: float = 1.0):
        """Draw classic high-contrast coordinate grid."""
        coords = np.arange(-size, size + step * 0.5, step)
        grid_color = [0.35, 0.38, 0.42]
        for x in coords:
            self.debug_item_ids.append(
                p.addUserDebugLine([float(x), -size, self.ground_z], [float(x), size, self.ground_z], grid_color, lineWidth=1.0, lifeTime=0, physicsClientId=self.client_id)
            )
        for y in coords:
            self.debug_item_ids.append(
                p.addUserDebugLine([-size, float(y), self.ground_z], [size, float(y), self.ground_z], grid_color, lineWidth=1.0, lifeTime=0, physicsClientId=self.client_id)
            )

    def close(self):
        """Clean up arena bodies and visual items from PyBullet."""
        for b_id in self.body_ids:
            try:
                p.removeBody(b_id, physicsClientId=self.client_id)
            except Exception:
                pass
        self.body_ids.clear()

        for d_id in self.debug_item_ids:
            try:
                p.removeUserDebugItem(d_id, physicsClientId=self.client_id)
            except Exception:
                pass
        self.debug_item_ids.clear()
