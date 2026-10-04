"""High-fidelity 3D visualization utilities for PyBullet simulation.

Integrates realistic arena ground, 7-gate RATM racing circuits, FPV OSD HUD overlays,
and multi-camera director (Chase, FPV Cockpit, Overview, Free).
"""

import time
import warnings
from typing import Optional, Callable, Dict, Any, List
import numpy as np
import pybullet as p

from .arena_scene import ArenaScene
from .race_gates import RaceGateManager
from .fpv_osd import FpvOsd
from .live_dashboard import LiveDashboard
from .live_telemetry import VisualizationSnapshot


class PyBulletVisualizer:
    """Manages 3D graphics, arena ground, racing gates, camera director, and HUD for PyBullet."""

    def __init__(
        self,
        client_id: int,
        uav_id: int,
        ground_z: float = 0.0,
        sim_speed: float = 1.0,
        enable_pacing: bool = True,
        ground_style: str = "arena",
        gates_mode: str = "lemniscate",
        cam_mode: str = "chase",
        enable_osd: bool = False,
        dashboard_enabled: bool = False,
    ):
        self.client_id = client_id
        self.uav_id = uav_id
        self.ground_z = float(ground_z)
        self.sim_speed = max(0.1, float(sim_speed))
        self.enable_pacing = bool(enable_pacing)
        self.ground_style = str(ground_style).lower()
        self.gates_mode = str(gates_mode).lower()
        self.cam_mode = str(cam_mode).lower()
        self.enable_osd = bool(enable_osd)

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
        self.cam_modes = ["chase", "fpv", "overview", "free"]
        if self.cam_mode not in self.cam_modes:
            self.cam_mode = "chase"
        self.cam_target = np.array([0.0, 0.0, 0.5], dtype=float)
        self.cam_dist = 2.6
        self.cam_yaw = 45.0
        self.cam_pitch = -20.0
        self.last_cam_update_step = 0

        # Flown trail state
        self.prev_trail_pos: Optional[np.ndarray] = None
        self.trail_points: List[np.ndarray] = []
        self.trail_ids: List[int] = []
        self.max_trail_segments = 600
        self.min_trail_dist = 0.03  # 3 cm decimation

        # Tracking previous drone position for gate traversal checks
        self.prev_drone_pos: Optional[np.ndarray] = None

        # Ground drop-line ID
        self.drop_line_id: Optional[int] = None

        # Setup Scene, Arena Ground, Gates, and OSD
        self._setup_scene()
        self.arena_scene = ArenaScene(
            client_id=self.client_id,
            ground_z=self.ground_z,
            style=self.ground_style,
        )
        self.gate_manager = RaceGateManager(client_id=self.client_id, ground_z=self.ground_z)
        self.fpv_osd = FpvOsd(client_id=self.client_id, enabled=self.enable_osd)

    def _setup_scene(self):
        """Configure shadows, disable clutter panels, and attach body triad."""
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=self.client_id)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1, physicsClientId=self.client_id)
        p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 1, physicsClientId=self.client_id)

        # Setup body triad (RGB axes attached directly to vehicle)
        self._setup_body_triad()

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
        """Pre-render the complete desired 3D path in sky-cyan and spawn racing gates."""
        times = np.arange(0.0, duration + dt_sample * 0.5, dt_sample)
        pts = [trajectory_fn(t)["H"][0:3, 3] for t in times]

        path_color = [0.72, 0.30, 1.0]  # Electric Neon Violet (distinct from cyan props and blue/orange gates)
        for i in range(len(pts) - 1):
            p.addUserDebugLine(
                pts[i].tolist(),
                pts[i + 1].tolist(),
                lineColorRGB=path_color,
                lineWidth=3.0,
                lifeTime=0,
                physicsClientId=self.client_id,
            )

        # Spawn Racing Gate Passages (aligned with world coordinates where trajectory resides)
        z_offset = 0.0
        if self.gates_mode == "lemniscate":
            self.gate_manager.load_lemniscate_4gates(z_offset=z_offset)
        elif self.gates_mode == "ratm":
            self.gate_manager.load_ratm_track(z_offset=z_offset)
        elif self.gates_mode == "auto":
            self.gate_manager.generate_path_gates(trajectory_fn, duration, num_gates=4)

    def _handle_keyboard(self):
        """Handle interactive keyboard hotkeys in PyBullet GUI."""
        keys = p.getKeyboardEvents(physicsClientId=self.client_id)
        for key_code, key_state in keys.items():
            if key_state & p.KEY_WAS_TRIGGERED:
                # Key 'C' (ord 99 / 67): Cycle camera modes
                if key_code in (ord('c'), ord('C')):
                    idx = self.cam_modes.index(self.cam_mode)
                    self.cam_mode = self.cam_modes[(idx + 1) % len(self.cam_modes)]
                # Key 'O' (ord 111 / 79): Toggle FPV OSD
                elif key_code in (ord('o'), ord('O')):
                    self.fpv_osd.toggle()
                # Key 'G' (ord 103 / 71): Toggle Gates
                elif key_code in (ord('g'), ord('G')):
                    if self.gate_manager.gates:
                        self.gate_manager.clear()
                    else:
                        z_off = 0.0
                        if self.gates_mode == "lemniscate":
                            self.gate_manager.load_lemniscate_4gates(z_offset=z_off)
                        else:
                            self.gate_manager.load_ratm_track(z_offset=z_off)

    def update(
        self,
        t: float,
        pos_err: float,
        s_norm: float,
        est_m: float,
        true_m: float,
        step_idx: int,
        mode: str = "BREGMAN",
        coriolis: str = "LC",
    ):
        """Update smooth camera tracking, gate traversals, trail, FPV OSD, and pacing."""
        pos, quat = p.getBasePositionAndOrientation(self.uav_id, physicsClientId=self.client_id)
        vel_lin, vel_ang = p.getBaseVelocity(self.uav_id, physicsClientId=self.client_id)
        pos_arr = np.array(pos, dtype=float)
        R = np.array(p.getMatrixFromQuaternion(quat), dtype=float).reshape(3, 3)

        # 1. Update flown trail (Vivid Hot Pink / Magenta, distinct from yellow boundary and orange gates)
        if self.prev_trail_pos is None:
            self.prev_trail_pos = pos_arr
        else:
            dist = np.linalg.norm(pos_arr - self.prev_trail_pos)
            if dist >= self.min_trail_dist:
                p.addUserDebugLine(
                    self.prev_trail_pos.tolist(),
                    pos_arr.tolist(),
                    lineColorRGB=[1.0, 0.15, 0.60],  # Vivid Hot Pink / Magenta
                    lineWidth=2.5,
                    lifeTime=0,
                    physicsClientId=self.client_id,
                )
                self.prev_trail_pos = pos_arr

        # 2. Check Racing Gate Traversal
        traversal = self.gate_manager.check_traversals(
            drone_pos=pos_arr,
            prev_pos=self.prev_drone_pos,
            t=t,
        )
        if traversal is not None:
            gate_id, split_time = traversal
            self.fpv_osd.trigger_gate_cleared(
                gate_id=gate_id,
                split_time=split_time,
                total_gates=len(self.gate_manager.gates),
            )
        self.prev_drone_pos = pos_arr.copy()

        # 3. Check Keyboard Events
        self._handle_keyboard()

        # 4. Multi-Camera Director (Updated smoothly on every visualization step)
        self.last_cam_update_step = step_idx
        self._update_camera_director(pos_arr, R)

        # 5. FPV Racing OSD & HUD Overlay (Updated on every visualization step at ~30 Hz)
        # Ground drop shadow line (shows altitude above floor)
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

        self.fpv_osd.update(
            t=t,
            pos=pos_arr,
            R=R,
            vel=np.array(vel_lin, dtype=float),
            pos_err=pos_err,
            s_norm=s_norm,
            est_m=est_m,
            true_m=true_m,
            mode=mode,
            coriolis=coriolis,
            sim_speed=self.sim_speed,
        )

        # 7. Wall-Clock Real-Time Pacing
        self._pace(t)

    def _update_camera_director(self, pos_arr: np.ndarray, R: np.ndarray):
        """Update PyBullet camera view according to active camera mode."""
        alpha = 0.08
        self.cam_target = (1.0 - alpha) * self.cam_target + alpha * pos_arr

        if self.cam_mode == "chase":
            # Third-person smooth follow
            cam_info = p.getDebugVisualizerCamera(physicsClientId=self.client_id)
            user_yaw = cam_info[8] if (cam_info and len(cam_info) >= 11) else self.cam_yaw
            user_pitch = cam_info[9] if (cam_info and len(cam_info) >= 11) else self.cam_pitch
            user_dist = cam_info[10] if (cam_info and len(cam_info) >= 11) else self.cam_dist

            p.resetDebugVisualizerCamera(
                cameraDistance=user_dist,
                cameraYaw=user_yaw,
                cameraPitch=user_pitch,
                cameraTargetPosition=self.cam_target.tolist(),
                physicsClientId=self.client_id,
            )

        elif self.cam_mode == "fpv":
            # Cockpit view placed at front FPV camera with 25 deg upward tilt
            fwd = R[:, 0]
            up = R[:, 2]
            drone_yaw_deg = float(np.degrees(np.arctan2(fwd[1], fwd[0])))
            drone_pitch_deg = float(np.degrees(np.arcsin(np.clip(-fwd[2], -1.0, 1.0))))
            cam_target_fpv = pos_arr + 2.0 * fwd

            p.resetDebugVisualizerCamera(
                cameraDistance=0.01,
                cameraYaw=drone_yaw_deg - 90.0,
                cameraPitch=drone_pitch_deg + 25.0,  # 25 deg FPV upward angle
                cameraTargetPosition=cam_target_fpv.tolist(),
                physicsClientId=self.client_id,
            )

        elif self.cam_mode == "overview":
            # Arena overview showing entire track
            p.resetDebugVisualizerCamera(
                cameraDistance=16.5,
                cameraYaw=35.0,
                cameraPitch=-35.0,
                cameraTargetPosition=[0.0, 0.0, 2.0],
                physicsClientId=self.client_id,
            )

        elif self.cam_mode == "free":
            # Free user control
            cam_info = p.getDebugVisualizerCamera(physicsClientId=self.client_id)
            if cam_info and len(cam_info) >= 11:
                p.resetDebugVisualizerCamera(
                    cameraDistance=cam_info[10],
                    cameraYaw=cam_info[8],
                    cameraPitch=cam_info[9],
                    cameraTargetPosition=self.cam_target.tolist(),
                    physicsClientId=self.client_id,
                )

    def update_snapshot(
        self,
        snapshot: VisualizationSnapshot,
        step_idx: int,
        mode: str = "BREGMAN",
        coriolis: str = "LC",
    ):
        """Render a scheduled snapshot, update 3D GUI, and publish to dashboard."""
        self.update(
            t=snapshot.t,
            pos_err=snapshot.position_error_norm,
            s_norm=snapshot.sliding_norm,
            est_m=float(snapshot.estimate_pi[0]),
            true_m=float(snapshot.true_pi[0]),
            step_idx=step_idx,
            mode=mode,
            coriolis=coriolis,
        )
        if self.dashboard is not None:
            self.dashboard.publish(snapshot)

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

    def close(self):
        """Clean up scene, gates, OSD, and child dashboard resources."""
        if hasattr(self, "arena_scene") and self.arena_scene is not None:
            self.arena_scene.close()
            self.arena_scene = None
        if hasattr(self, "gate_manager") and self.gate_manager is not None:
            self.gate_manager.close()
            self.gate_manager = None
        if hasattr(self, "fpv_osd") and self.fpv_osd is not None:
            self.fpv_osd.close()
            self.fpv_osd = None
        if self.dashboard is not None:
            self.dashboard.close()
            self.dashboard = None
