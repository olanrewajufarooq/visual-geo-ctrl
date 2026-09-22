"""PyBullet simulation plant for floating-base UAV with adaptive control."""

import os
import tempfile
from typing import Optional, Dict, Any, Tuple
import numpy as np
from .suppress import suppress_c_stdout

with suppress_c_stdout():
    import pybullet as p
    import pybullet_data

from ..math.se3 import quat_to_rotm, rotm_to_quat, skew
from ..math.inertia import inertia_from_pi
from .drone_urdf import generate_multicopter_urdf
from ..viz.pybullet_viz import PyBulletVisualizer
from ..viz.live_telemetry import VisualizationSnapshot


class PyBulletPlant:
    """Floating-base UAV physical plant simulated via PyBullet."""

    def __init__(
        self,
        pi: np.ndarray,
        gravity: np.ndarray = np.array([0.0, 0.0, -9.81]),
        dt: float = 0.002,
        gui: bool = False,
        payload: Optional[Dict[str, Any]] = None,
        release_time: Optional[float] = None,
        sim_speed: float = 1.0,
        enable_pacing: bool = True,
        ground_z: float = -1.5,
        ground_style: str = "arena",
        gates_mode: str = "lemniscate",
        cam_mode: str = "chase",
        enable_osd: bool = True,
        drone_type: str = "pybullet_drones",
    ):
        self.dt = float(dt)
        self.gravity = np.asarray(gravity, dtype=float).ravel()
        self.gui = gui
        self.payload_info = payload
        self.release_time = release_time
        self.payload_dropped = False
        self.sim_speed = float(sim_speed)
        self.enable_pacing = bool(enable_pacing)
        self.ground_z = float(ground_z)
        self.ground_style = ground_style
        self.gates_mode = gates_mode
        self.cam_mode = cam_mode
        self.enable_osd = enable_osd
        self.drone_type = str(drone_type)

        # Connect to PyBullet
        connection_mode = p.GUI if self.gui else p.DIRECT
        with suppress_c_stdout():
            self.client_id = p.connect(connection_mode)
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.client_id)
        p.setGravity(self.gravity[0], self.gravity[1], self.gravity[2], physicsClientId=self.client_id)
        p.setTimeStep(self.dt, physicsClientId=self.client_id)
        p.setRealTimeSimulation(0, physicsClientId=self.client_id)

        if self.gui:
            # Add plane URDF if plane style is requested or as invisible baseline collision
            with suppress_c_stdout():
                self.plane_id = p.loadURDF("plane.urdf", [0, 0, self.ground_z], physicsClientId=self.client_id)
        else:
            self.plane_id = None

        # Create UAV rigid body with realistic multicopter URDF
        self.pi = np.asarray(pi, dtype=float).ravel()
        self.uav_id, self.r_com = self._create_uav_body(self.pi)

        # Create Visualizer in GUI mode
        if self.gui:
            self.visualizer: Optional[PyBulletVisualizer] = PyBulletVisualizer(
                client_id=self.client_id,
                uav_id=self.uav_id,
                ground_z=self.ground_z,
                sim_speed=self.sim_speed,
                ground_style=self.ground_style,
                gates_mode=self.gates_mode,
                cam_mode=self.cam_mode,
                enable_osd=self.enable_osd,
                dashboard_enabled=True,
            )
        else:
            self.visualizer = None

        # Create Payload if specified
        self.payload_id = None
        self.constraint_id = None
        if self.payload_info is not None:
            self._setup_payload(self.payload_info)

        # Legacy HUD text ID
        self.debug_text_id = None

    def _create_uav_body(self, pi: np.ndarray) -> Tuple[int, np.ndarray]:
        """Create UAV multibody with exact inertial parameters and multicopter visuals."""
        m = float(pi[0])
        h = pi[1:4]
        r_com = h / m

        # Generate temporary URDF with exact physical parameters
        tmp_fd, tmp_path = tempfile.mkstemp(suffix=".urdf", prefix="uav_drone_")
        os.close(tmp_fd)
        try:
            generate_multicopter_urdf(pi, tmp_path, drone_type=self.drone_type)
            with suppress_c_stdout():
                uav_id = p.loadURDF(
                    tmp_path,
                    basePosition=[0.0, 0.0, 0.0],
                    baseOrientation=[0.0, 0.0, 0.0, 1.0],
                    flags=p.URDF_MERGE_FIXED_LINKS,
                    physicsClientId=self.client_id,
                )
        finally:
            if os.path.exists(tmp_path):
                try:
                    os.remove(tmp_path)
                except OSError:
                    pass

        # Remove damping to match paper free-body dynamics
        p.changeDynamics(
            uav_id,
            -1,
            linearDamping=0.0,
            angularDamping=0.0,
            physicsClientId=self.client_id,
        )
        return uav_id, r_com

    def _setup_payload(self, payload: dict):
        """Create delivery payload rigidly attached to vehicle."""
        m_p = float(payload['mass'])
        d_p = np.asarray(payload['dimensions'], dtype=float)
        center = np.asarray(payload['center'], dtype=float)

        col_shape = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=(d_p / 2.0).tolist(),
            physicsClientId=self.client_id,
        )
        # High-visibility delivery payload styling (hazard orange)
        vis_shape = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=(d_p / 2.0).tolist(),
            rgbaColor=[0.95, 0.45, 0.05, 0.95],
            specularColor=[0.9, 0.9, 0.9],
            physicsClientId=self.client_id,
        )

        self.payload_id = p.createMultiBody(
            baseMass=m_p,
            baseCollisionShapeIndex=col_shape,
            baseVisualShapeIndex=vis_shape,
            basePosition=center.tolist(),
            baseOrientation=[0.0, 0.0, 0.0, 1.0],
            physicsClientId=self.client_id,
        )
        p.changeDynamics(
            self.payload_id,
            -1,
            linearDamping=0.0,
            angularDamping=0.0,
            lateralFriction=0.8,
            restitution=0.2,
            physicsClientId=self.client_id,
        )

        # Rigidly attach payload to vehicle body
        self.constraint_id = p.createConstraint(
            parentBodyUniqueId=self.uav_id,
            parentLinkIndex=-1,
            childBodyUniqueId=self.payload_id,
            childLinkIndex=-1,
            jointType=p.JOINT_FIXED,
            jointAxis=[0.0, 0.0, 0.0],
            parentFramePosition=center.tolist(),
            childFramePosition=[0.0, 0.0, 0.0],
            physicsClientId=self.client_id,
        )

    def draw_reference_path(self, trajectory_fn, duration: float):
        """Pre-render reference trajectory if visualizer is active."""
        if self.visualizer is not None:
            self.visualizer.draw_reference_path(trajectory_fn, duration)

    def update_viz(
        self,
        snapshot: Optional[VisualizationSnapshot] = None,
        step_idx: int = 0,
        mode: str = "BREGMAN",
        coriolis: str = "C1",
        **kwargs,
    ):
        """Update scheduled 3D and dashboard visualization."""
        if self.visualizer is not None:
            if snapshot is not None:
                self.visualizer.update_snapshot(snapshot, step_idx, mode=mode, coriolis=coriolis)
            else:
                self.visualizer.update(
                    t=kwargs.get("t", 0.0),
                    pos_err=kwargs.get("pos_err", 0.0),
                    s_norm=kwargs.get("s_norm", 0.0),
                    est_m=kwargs.get("est_m", float(self.pi[0])),
                    true_m=kwargs.get("true_m", float(self.pi[0])),
                    payload_dropped=self.payload_dropped,
                    step_idx=step_idx,
                    mode=mode,
                    coriolis=coriolis,
                )

    def set_state(self, H: np.ndarray, V: np.ndarray):
        """Set floating UAV state in PyBullet."""
        R = H[0:3, 0:3]
        pos = H[0:3, 3].tolist()
        quat = rotm_to_quat(R).tolist()

        # Body twist V = [omega_b; v_b] -> world velocities
        omega_w = (R @ V[0:3]).tolist()
        v_w = (R @ V[3:6]).tolist()

        p.resetBasePositionAndOrientation(
            self.uav_id, pos, quat, physicsClientId=self.client_id
        )
        p.resetBaseVelocity(
            self.uav_id, v_w, omega_w, physicsClientId=self.client_id
        )

        if self.payload_id is not None and self.constraint_id is not None:
            r_w = R @ self.payload_info['center']
            p_pay = (H[0:3, 3] + r_w).tolist()
            p.resetBasePositionAndOrientation(
                self.payload_id, p_pay, quat, physicsClientId=self.client_id
            )
            v_pay_w = (R @ (V[3:6] + skew(V[0:3]) @ self.payload_info['center'])).tolist()
            p.resetBaseVelocity(
                self.payload_id, v_pay_w, omega_w, physicsClientId=self.client_id
            )

    def get_state(self) -> Dict[str, np.ndarray]:
        """Read current state in paper convention: H in SE(3), V = [omega_b; v_b]."""
        pos, quat = p.getBasePositionAndOrientation(self.uav_id, physicsClientId=self.client_id)
        v_w, omega_w = p.getBaseVelocity(self.uav_id, physicsClientId=self.client_id)

        R = quat_to_rotm(np.array(quat))
        p_vec = np.array(pos)
        v_w = np.array(v_w)
        omega_w = np.array(omega_w)

        # Left-trivialized body twist
        omega_b = R.T @ omega_w
        v_b = R.T @ v_w

        H = np.eye(4, dtype=float)
        H[0:3, 0:3] = R
        H[0:3, 3] = p_vec
        V = np.concatenate([omega_b, v_b])
        return {"H": H, "V": V}

    def apply_wrench(self, wrench: np.ndarray):
        """Apply commanded body wrench W = [tau_b; f_b] at body origin in link frame."""
        wrench = np.asarray(wrench, dtype=float).ravel()
        tau_b = wrench[0:3].tolist()
        f_b = wrench[3:6].tolist()

        p.applyExternalForce(
            objectUniqueId=self.uav_id,
            linkIndex=-1,
            forceObj=f_b,
            posObj=[0.0, 0.0, 0.0],
            flags=p.LINK_FRAME,
            physicsClientId=self.client_id,
        )
        p.applyExternalTorque(
            objectUniqueId=self.uav_id,
            linkIndex=-1,
            torqueObj=tau_b,
            flags=p.LINK_FRAME,
            physicsClientId=self.client_id,
        )

    def check_payload_drop(self, time: float):
        """Release payload physically at releaseTime."""
        if (
            not self.payload_dropped
            and self.release_time is not None
            and time >= self.release_time
            and self.constraint_id is not None
        ):
            p.removeConstraint(self.constraint_id, physicsClientId=self.client_id)
            self.constraint_id = None
            self.payload_dropped = True

    def step(self, time: Optional[float] = None):
        """Advance physical simulation by dt."""
        if time is not None:
            self.check_payload_drop(time)

        p.stepSimulation(physicsClientId=self.client_id)

    def update_hud(self, text: str):
        """Legacy HUD updater."""
        if self.gui:
            if self.debug_text_id is not None:
                p.removeUserDebugItem(self.debug_text_id, physicsClientId=self.client_id)
            pos, _ = p.getBasePositionAndOrientation(self.uav_id, physicsClientId=self.client_id)
            self.debug_text_id = p.addUserDebugText(
                text,
                [pos[0], pos[1], pos[2] + 0.4],
                textColorRGB=[1.0, 1.0, 0.2],
                textSize=1.1,
                physicsClientId=self.client_id,
            )

    def close(self):
        """Disconnect PyBullet session and clean up visualizer."""
        if hasattr(self, "visualizer") and self.visualizer is not None:
            self.visualizer.close()
            self.visualizer = None
        if p.isConnected(physicsClientId=self.client_id):
            p.disconnect(physicsClientId=self.client_id)

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass
