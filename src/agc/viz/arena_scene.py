"""Realistic indoor flight arena environment and ground styling for PyBullet.

Digital twin of the TII indoor flight arena (25m x 10m x 6m) used in:
- Bosello et al., "Race Against the Machine", IEEE RA-L 2024.
- Bosello et al., "On Your Own", IEEE RA-L 2026.

Uses static URDF scene loading from disk (assets/arena/arena_scene.urdf) to eliminate
per-second visual redraws and flickering.
"""

from pathlib import Path
from typing import List, Optional
import numpy as np
import pybullet as p


class ArenaScene:
    """Manages the visual flight arena floor, launch pad, safety boundaries, and enclosure."""

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
            repo_root = Path(__file__).resolve().parents[3]
            arena_urdf = (repo_root / "assets" / "arena" / "arena_scene.urdf").resolve()
            if arena_urdf.exists():
                arena_id = p.loadURDF(
                    str(arena_urdf).replace("\\", "/"),
                    basePosition=[0.0, 0.0, self.ground_z],
                    baseOrientation=[0.0, 0.0, 0.0, 1.0],
                    useFixedBase=True,
                    flags=p.URDF_MERGE_FIXED_LINKS,
                    physicsClientId=self.client_id,
                )
                self.body_ids.append(arena_id)
            else:
                self._build_procedural_fallback(half_l, half_w)
        elif self.style == "grid":
            self._build_coordinate_grid(size=max(half_l, half_w), step=1.0)
        # style "plane" leaves default plane only

    def _build_procedural_fallback(self, half_l: float, half_w: float):
        """Procedural geometric boxes fallback if URDF file is missing."""
        vis_floor = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[half_l, half_w, 0.005],
            rgbaColor=[0.14, 0.15, 0.17, 1.0],
            physicsClientId=self.client_id,
        )
        col_floor = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[half_l, half_w, 0.005],
            physicsClientId=self.client_id,
        )
        floor_id = p.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=col_floor,
            baseVisualShapeIndex=vis_floor,
            basePosition=[0.0, 0.0, self.ground_z - 0.005],
            physicsClientId=self.client_id,
        )
        self.body_ids.append(floor_id)

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
