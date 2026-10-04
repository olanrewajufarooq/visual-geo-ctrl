"""PyBullet simulation plant for the nominal floating-base UAV."""

import os
import tempfile
from typing import Optional, Tuple

import numpy as np

from .suppress import suppress_c_stdout

with suppress_c_stdout():
    import pybullet as p
    import pybullet_data

from ..math.se3 import quat_to_rotm, rotm_to_quat
from .drone_urdf import generate_multicopter_urdf
from ..viz.pybullet_viz import PyBulletVisualizer
from ..viz.live_telemetry import VisualizationSnapshot


class PyBulletPlant:
    """Bare floating-base UAV physical plant simulated via PyBullet."""

    def __init__(self, pi, gravity=np.array([0.0, 0.0, -9.81]), dt=0.002, gui=False,
                 sim_speed=1.0, enable_pacing=True, ground_z=0.0, ground_style="arena",
                 gates_mode="lemniscate", cam_mode="chase", enable_osd=False,
                 drone_type="pybullet_drones"):
        self.dt = float(dt)
        self.gravity = np.asarray(gravity, dtype=float).ravel()
        self.gui = bool(gui)
        self.sim_speed = float(sim_speed)
        self.enable_pacing = bool(enable_pacing)
        self.ground_z = float(ground_z)
        self.ground_style = ground_style
        self.gates_mode = gates_mode
        self.cam_mode = cam_mode
        self.enable_osd = enable_osd
        self.drone_type = str(drone_type)
        with suppress_c_stdout():
            self.client_id = p.connect(p.GUI if self.gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.client_id)
        p.setGravity(*self.gravity[:3], physicsClientId=self.client_id)
        p.setTimeStep(self.dt, physicsClientId=self.client_id)
        p.setRealTimeSimulation(0, physicsClientId=self.client_id)
        if self.gui and self.ground_style == "plane":
            with suppress_c_stdout():
                self.plane_id = p.loadURDF("plane.urdf", [0, 0, self.ground_z], physicsClientId=self.client_id)
        else:
            self.plane_id = None
        self.pi = np.asarray(pi, dtype=float).ravel()
        self.uav_id, self.r_com = self._create_uav_body(self.pi)
        dynamics = p.getDynamicsInfo(self.uav_id, -1, physicsClientId=self.client_id)
        self.r_com = np.asarray(dynamics[3])
        self.R_inertial = quat_to_rotm(np.asarray(dynamics[4]))
        self.visualizer = PyBulletVisualizer(
            client_id=self.client_id, uav_id=self.uav_id, ground_z=self.ground_z,
            sim_speed=self.sim_speed, ground_style=self.ground_style, gates_mode=self.gates_mode,
            cam_mode=self.cam_mode, enable_osd=self.enable_osd, dashboard_enabled=True,
        ) if self.gui else None
        self.debug_text_id = None

    def _create_uav_body(self, pi: np.ndarray) -> Tuple[int, np.ndarray]:
        mass = float(pi[0])
        r_com = pi[1:4] / mass
        tmp_fd, tmp_path = tempfile.mkstemp(suffix=".urdf", prefix="uav_drone_")
        os.close(tmp_fd)
        try:
            generate_multicopter_urdf(pi, tmp_path, drone_type=self.drone_type)
            with suppress_c_stdout():
                uav_id = p.loadURDF(tmp_path, basePosition=[0, 0, 0], baseOrientation=[0, 0, 0, 1],
                                    flags=p.URDF_USE_INERTIA_FROM_FILE, physicsClientId=self.client_id)
        finally:
            if os.path.exists(tmp_path):
                os.remove(tmp_path)
        p.changeDynamics(uav_id, -1, linearDamping=0.0, angularDamping=0.0, physicsClientId=self.client_id)
        return uav_id, r_com

    def draw_reference_path(self, trajectory_fn, duration):
        if self.visualizer is not None:
            self.visualizer.draw_reference_path(trajectory_fn, duration)

    def update_viz(self, snapshot: Optional[VisualizationSnapshot] = None, step_idx=0,
                   mode="nominal", coriolis="lc", **kwargs):
        if self.visualizer is not None:
            if snapshot is not None:
                self.visualizer.update_snapshot(snapshot, step_idx, mode=mode, coriolis=coriolis)
            else:
                self.visualizer.update(t=kwargs.get("t", 0.0), pos_err=kwargs.get("pos_err", 0.0),
                    s_norm=kwargs.get("s_norm", 0.0), est_m=float(self.pi[0]), true_m=float(self.pi[0]),
                    step_idx=step_idx, mode=mode, coriolis=coriolis)

    def set_state(self, H, V):
        R = H[:3, :3]
        pos = (H[:3, 3] + R @ self.r_com).tolist()
        quat = rotm_to_quat(R @ self.R_inertial).tolist()
        p.resetBasePositionAndOrientation(self.uav_id, pos, quat, physicsClientId=self.client_id)
        p.resetBaseVelocity(self.uav_id, (R @ (V[3:6] + np.cross(V[:3], self.r_com))).tolist(),
                            (R @ V[:3]).tolist(), physicsClientId=self.client_id)

    def get_state(self):
        pos, quat = p.getBasePositionAndOrientation(self.uav_id, physicsClientId=self.client_id)
        v_w, omega_w = p.getBaseVelocity(self.uav_id, physicsClientId=self.client_id)
        R = quat_to_rotm(np.array(quat)) @ self.R_inertial.T
        omega_b = R.T @ np.asarray(omega_w)
        v_b = R.T @ np.asarray(v_w) - np.cross(omega_b, self.r_com)
        H = np.eye(4)
        H[:3, :3] = R
        H[:3, 3] = np.asarray(pos) - R @ self.r_com
        return {"H": H, "V": np.concatenate([omega_b, v_b])}

    def apply_wrench(self, wrench):
        state = self.get_state()
        R = state["H"][:3, :3]
        p.applyExternalForce(self.uav_id, -1, (R @ np.asarray(wrench[3:6])).tolist(),
                             state["H"][:3, 3].tolist(), p.WORLD_FRAME, physicsClientId=self.client_id)
        p.applyExternalTorque(self.uav_id, -1, (R @ np.asarray(wrench[:3])).tolist(),
                              p.WORLD_FRAME, physicsClientId=self.client_id)

    def step(self, time=None):
        p.stepSimulation(physicsClientId=self.client_id)

    def update_hud(self, text):
        if self.gui:
            if self.debug_text_id is not None:
                p.removeUserDebugItem(self.debug_text_id, physicsClientId=self.client_id)
            pos, _ = p.getBasePositionAndOrientation(self.uav_id, physicsClientId=self.client_id)
            self.debug_text_id = p.addUserDebugText(text, [pos[0], pos[1], pos[2] + 0.4],
                                                     textColorRGB=[1, 1, 0.2], textSize=1.1,
                                                     physicsClientId=self.client_id)

    def close(self):
        if self.visualizer is not None:
            self.visualizer.close()
            self.visualizer = None
        if p.isConnected(physicsClientId=self.client_id):
            p.disconnect(physicsClientId=self.client_id)
        self.client_id = -1

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass
