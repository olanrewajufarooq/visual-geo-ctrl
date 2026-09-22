"""High-fidelity 3D visualization utilities for PyBullet simulation."""

import time
import warnings
from typing import Optional, Callable, Dict, Any, List
import numpy as np
import pybullet as p

from .live_dashboard import LiveDashboard
from .live_telemetry import VisualizationSnapshot


class PyBulletVisualizer:
    """Manages 3D graphics, ground grid, camera, trajectories, and HUD for PyBullet."""

    def __init__(
        self,
        client_id: int,
        uav_id: int,
        ground_z: float = -1.5,
        sim_speed: float = 1.0,
        enable_pacing: bool = True,
        dashboard_enabled: bool = False,
    ):
        self.client_id = client_id
        self.uav_id = uav_id
        self.ground_z = float(ground_z)
        self.sim_speed = max(0.1, float(sim_speed))
        self.enable_pacing = bool(enable_pacing)
        self.dashboard: Optional[LiveDashboard] = None
        if dashboard_enabled:
            try:
                self.dashboard = LiveDashboard()
            except Exception as exc:
                warnings.warn(
                    f"Live dashboard could not be started; continuing with PyBullet-only GUI: {exc}",
                    RuntimeWarning,
                    stacklevel=2,
                )

        # Real-time pacing clock
        self.wall_start_time: Optional[float] = None
        self.sim_start_time: Optional[float] = None

        # Camera state
        self.cam_target = np.array([0.0, 0.0, 0.5], dtype=float)
        self.cam_dist = 2.8
        self.cam_yaw = 45.0
        self.cam_pitch = -25.0
        self.last_cam_update_step = 0

        # Flown trail state
        self.prev_trail_pos: Optional[np.ndarray] = None
        self.trail_points: List[np.ndarray] = []
        self.trail_ids: List[int] = []
        self.max_trail_segments = 600
        self.min_trail_dist = 0.03  # 3 cm decimation

        # Ground drop-line ID
        self.drop_line_id: Optional[int] = None

        # HUD text ID
        self.hud_id: Optional[int] = None

        # Setup environment
        self._setup_scene()

    def _setup_scene(self):
        """Configure shadows, disable clutter panels, and draw ground grid."""
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=self.client_id)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1, physicsClientId=self.client_id)
        p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 1, physicsClientId=self.client_id)

        # Draw coordinate ground grid
        self._draw_ground_grid(size=8.0, step=1.0)

        # Setup body triad (RGB axes attached directly to vehicle)
        self._setup_body_triad()

    def _draw_ground_grid(self, size: float = 8.0, step: float = 1.0):
        """Draw an elegant high-contrast ground grid and coordinate axes at ground_z."""
        coords = np.arange(-size, size + step * 0.5, step)
        grid_color = [0.35, 0.38, 0.42]  # Slate gray
        axis_color_x = [0.75, 0.25, 0.25]  # Muted red for X ground axis
        axis_color_y = [0.25, 0.75, 0.25]  # Muted green for Y ground axis

        # Grid lines parallel to Y
        for x in coords:
            col = axis_color_y if abs(x) < 1e-4 else grid_color
            width = 2.0 if abs(x) < 1e-4 else 1.0
            p.addUserDebugLine(
                [float(x), -size, self.ground_z],
                [float(x), size, self.ground_z],
                lineColorRGB=col,
                lineWidth=width,
                lifeTime=0,
                physicsClientId=self.client_id,
            )

        # Grid lines parallel to X
        for y in coords:
            col = axis_color_x if abs(y) < 1e-4 else grid_color
            width = 2.0 if abs(y) < 1e-4 else 1.0
            p.addUserDebugLine(
                [-size, float(y), self.ground_z],
                [size, float(y), self.ground_z],
                lineColorRGB=col,
                lineWidth=width,
                lifeTime=0,
                physicsClientId=self.client_id,
            )

        # Origin landing circle/cross
        r = 0.5
        n_seg = 24
        angles = np.linspace(0, 2 * np.pi, n_seg + 1)
        for i in range(n_seg):
            p1 = [r * np.cos(angles[i]), r * np.sin(angles[i]), self.ground_z + 0.005]
            p2 = [r * np.cos(angles[i + 1]), r * np.sin(angles[i + 1]), self.ground_z + 0.005]
            p.addUserDebugLine(
                p1, p2, lineColorRGB=[0.0, 0.8, 1.0], lineWidth=2.0, lifeTime=0, physicsClientId=self.client_id
            )

    def _setup_body_triad(self, axis_len: float = 0.35):
        """Attach local body frame triad (Red=X, Green=Y, Blue=Z) to vehicle link."""
        # Red = +X_b (Forward)
        p.addUserDebugLine(
            [0.0, 0.0, 0.0],
            [axis_len, 0.0, 0.0],
            lineColorRGB=[1.0, 0.1, 0.1],
            lineWidth=3.0,
            lifeTime=0,
            parentObjectUniqueId=self.uav_id,
            parentLinkIndex=-1,
            physicsClientId=self.client_id,
        )
        # Green = +Y_b (Left)
        p.addUserDebugLine(
            [0.0, 0.0, 0.0],
            [0.0, axis_len, 0.0],
            lineColorRGB=[0.1, 0.95, 0.2],
            lineWidth=3.0,
            lifeTime=0,
            parentObjectUniqueId=self.uav_id,
            parentLinkIndex=-1,
            physicsClientId=self.client_id,
        )
        # Blue = +Z_b (Thrust / Up)
        p.addUserDebugLine(
            [0.0, 0.0, 0.0],
            [0.0, 0.0, axis_len],
            lineColorRGB=[0.15, 0.45, 1.0],
            lineWidth=3.0,
            lifeTime=0,
            parentObjectUniqueId=self.uav_id,
            parentLinkIndex=-1,
            physicsClientId=self.client_id,
        )

    def draw_reference_path(
        self,
        trajectory_fn: Callable[[float], Dict[str, Any]],
        duration: float,
        dt_sample: float = 0.05,
    ):
        """Pre-render the complete desired 3D path in sky-cyan before simulation starts."""
        times = np.arange(0.0, duration + dt_sample * 0.5, dt_sample)
        pts = [trajectory_fn(t)["H"][0:3, 3] for t in times]

        path_color = [0.1, 0.75, 1.0]  # Sky cyan
        for i in range(len(pts) - 1):
            p.addUserDebugLine(
                pts[i].tolist(),
                pts[i + 1].tolist(),
                lineColorRGB=path_color,
                lineWidth=2.5,
                lifeTime=0,
                physicsClientId=self.client_id,
            )

        # Mark start location
        p.addUserDebugText(
            "START",
            [pts[0][0], pts[0][1], pts[0][2] + 0.15],
            textColorRGB=[0.2, 1.0, 0.3],
            textSize=1.1,
            lifeTime=0,
            physicsClientId=self.client_id,
        )

    def update(
        self,
        t: float,
        pos_err: float,
        s_norm: float,
        est_m: float,
        true_m: float,
        payload_dropped: bool,
        step_idx: int,
    ):
        """Perform smooth camera tracking, trail decimation, ground shadow, HUD, and pacing."""
        pos, _ = p.getBasePositionAndOrientation(self.uav_id, physicsClientId=self.client_id)
        pos_arr = np.array(pos, dtype=float)

        # 1. Update flown trail (amber/gold, decimated to prevent buffer lag)
        if self.prev_trail_pos is None:
            self.prev_trail_pos = pos_arr
        else:
            dist = np.linalg.norm(pos_arr - self.prev_trail_pos)
            if dist >= self.min_trail_dist:
                p.addUserDebugLine(
                    self.prev_trail_pos.tolist(),
                    pos_arr.tolist(),
                    lineColorRGB=[1.0, 0.78, 0.15],  # Amber gold
                    lineWidth=2.0,
                    lifeTime=0,
                    physicsClientId=self.client_id,
                )
                self.prev_trail_pos = pos_arr

        # 2. Ground drop shadow line (shows altitude above floor)
        shadow_start = pos_arr.tolist()
        shadow_end = [pos_arr[0], pos_arr[1], self.ground_z]
        if self.drop_line_id is None:
            self.drop_line_id = p.addUserDebugLine(
                shadow_start,
                shadow_end,
                lineColorRGB=[0.3, 0.3, 0.3],
                lineWidth=1.0,
                lifeTime=0,
                physicsClientId=self.client_id,
            )
        else:
            self.drop_line_id = p.addUserDebugLine(
                shadow_start,
                shadow_end,
                lineColorRGB=[0.3, 0.3, 0.3],
                lineWidth=1.0,
                lifeTime=0,
                replaceItemUniqueId=self.drop_line_id,
                physicsClientId=self.client_id,
            )

        # 3. Smooth Camera Tracking (60 Hz rate, preserves user mouse look/zoom)
        if step_idx - self.last_cam_update_step >= 8:  # ~60 Hz at 500 Hz physics
            self.last_cam_update_step = step_idx
            # Exponential smoothing on camera target
            alpha = 0.08
            self.cam_target = (1.0 - alpha) * self.cam_target + alpha * pos_arr

            # Check if user rotated/zoomed with mouse
            cam_info = p.getDebugVisualizerCamera(physicsClientId=self.client_id)
            if cam_info is not None and len(cam_info) >= 11:
                user_yaw = cam_info[8]
                user_pitch = cam_info[9]
                user_dist = cam_info[10]
            else:
                user_yaw = self.cam_yaw
                user_pitch = self.cam_pitch
                user_dist = self.cam_dist

            p.resetDebugVisualizerCamera(
                cameraDistance=user_dist,
                cameraYaw=user_yaw,
                cameraPitch=user_pitch,
                cameraTargetPosition=self.cam_target.tolist(),
                physicsClientId=self.client_id,
            )

        # 4. HUD Telemetry overlay (updated every 50 steps = 10 Hz)
        if step_idx % 50 == 0:
            status = "DROPPED" if payload_dropped else "ATTACHED"
            hud_text = (
                f"T = {t:5.2f}s | Speed: {self.sim_speed:.1f}x\n"
                f"||e_p|| = {pos_err:.3f} m\n"
                f"||s||   = {s_norm:.2f}\n"
                f"m_hat   = {est_m:.2f} kg (true: {true_m:.2f})\n"
                f"Payload: {status}"
            )
            hud_pos = [pos_arr[0], pos_arr[1], pos_arr[2] + 0.45]
            if self.hud_id is None:
                self.hud_id = p.addUserDebugText(
                    hud_text,
                    hud_pos,
                    textColorRGB=[1.0, 1.0, 0.3],
                    textSize=1.05,
                    lifeTime=0,
                    physicsClientId=self.client_id,
                )
            else:
                self.hud_id = p.addUserDebugText(
                    hud_text,
                    hud_pos,
                    textColorRGB=[1.0, 1.0, 0.3],
                    textSize=1.05,
                    lifeTime=0,
                    replaceItemUniqueId=self.hud_id,
                    physicsClientId=self.client_id,
                )

        # 5. Wall-Clock Real-Time Pacing
        self._pace(t)

    def update_snapshot(self, snapshot: VisualizationSnapshot, step_idx: int):
        """Render a scheduled snapshot and publish it without blocking simulation."""
        position = snapshot.actual_H[:3, 3]
        self._update_spatial_state(position, snapshot, step_idx)
        if self.dashboard is not None:
            self.dashboard.publish(snapshot)
        self._pace(snapshot.t)

    def _pace(self, t: float):
        """Pace GUI playback without coupling headless or fast runs to rendering."""
        if not self.enable_pacing:
            return
        now = time.perf_counter()
        if self.wall_start_time is None:
            self.wall_start_time = now
            self.sim_start_time = t
            return
        elapsed_sim = (t - self.sim_start_time) / self.sim_speed
        elapsed_wall = now - self.wall_start_time
        sleep_needed = elapsed_sim - elapsed_wall
        if sleep_needed > 0.001:
            time.sleep(sleep_needed)

    def _update_spatial_state(
        self,
        pos_arr: np.ndarray,
        snapshot: VisualizationSnapshot,
        step_idx: int,
    ):
        """Update bounded 3D annotations using an already sampled position."""
        if self.prev_trail_pos is None:
            self.prev_trail_pos = pos_arr.copy()
        else:
            dist = np.linalg.norm(pos_arr - self.prev_trail_pos)
            if dist >= self.min_trail_dist:
                trail_id = p.addUserDebugLine(
                    self.prev_trail_pos.tolist(),
                    pos_arr.tolist(),
                    lineColorRGB=[1.0, 0.78, 0.15],
                    lineWidth=2.0,
                    lifeTime=0,
                    physicsClientId=self.client_id,
                )
                self.trail_ids.append(trail_id)
                if len(self.trail_ids) > self.max_trail_segments:
                    old_id = self.trail_ids.pop(0)
                    p.removeUserDebugItem(old_id, physicsClientId=self.client_id)
                self.prev_trail_pos = pos_arr.copy()

        shadow_start = pos_arr.tolist()
        shadow_end = [pos_arr[0], pos_arr[1], self.ground_z]
        if self.drop_line_id is None:
            self.drop_line_id = p.addUserDebugLine(
                shadow_start,
                shadow_end,
                lineColorRGB=[0.3, 0.3, 0.3],
                lineWidth=1.0,
                lifeTime=0,
                physicsClientId=self.client_id,
            )
        else:
            self.drop_line_id = p.addUserDebugLine(
                shadow_start,
                shadow_end,
                lineColorRGB=[0.3, 0.3, 0.3],
                lineWidth=1.0,
                replaceItemUniqueId=self.drop_line_id,
                physicsClientId=self.client_id,
            )

        if step_idx - self.last_cam_update_step >= 8:
            self.last_cam_update_step = step_idx
            self.cam_target = 0.92 * self.cam_target + 0.08 * pos_arr
            cam_info = p.getDebugVisualizerCamera(physicsClientId=self.client_id)
            if cam_info is not None and len(cam_info) >= 11:
                user_yaw, user_pitch, user_dist = cam_info[8:11]
            else:
                user_yaw, user_pitch, user_dist = self.cam_yaw, self.cam_pitch, self.cam_dist
            p.resetDebugVisualizerCamera(
                cameraDistance=user_dist,
                cameraYaw=user_yaw,
                cameraPitch=user_pitch,
                cameraTargetPosition=self.cam_target.tolist(),
                physicsClientId=self.client_id,
            )

    def close(self):
        """Close child dashboard resources."""
        if self.dashboard is not None:
            self.dashboard.close()
            self.dashboard = None
