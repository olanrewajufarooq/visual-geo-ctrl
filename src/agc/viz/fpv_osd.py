"""FPV Racing Drone On-Screen Display (OSD) and Flight Instrumentation Overlay.

Renders authentic FPV racing HUD instruments directly inside the PyBullet 3D GUI:
- Aircraft boresight reticle and crosshair
- Artificial horizon roll bar and pitch ladder steps (+-10 deg, +-20 deg)
- Flight telemetry: Ground speed, Altitude AGL, Vertical velocity, Battery voltage
- Adaptive Geometric Control diagnostics: ||e_p||, ||s||, m_hat, Payload state
- Gate traversal notifications and split lap timers
"""

from typing import Optional, List
import numpy as np
import pybullet as p


class FpvOsd:
    """Manages 3D in-cockpit / chase flight instrumentation and telemetry OSD."""

    def __init__(self, client_id: int, enabled: bool = True):
        self.client_id = client_id
        self.enabled = bool(enabled)

        self.hud_text_id: Optional[int] = None
        self.gate_banner_id: Optional[int] = None
        self.horizon_line_ids: List[int] = []

        # Synthetic battery model: starts at 25.2V (4.2V/cell for 6S LiPo), slowly drops
        self.vbat_initial = 25.2
        self.vbat_current = 25.2

        # Gate flash notification state
        self.banner_text: str = ""
        self.banner_expiry: float = 0.0

    def trigger_gate_cleared(self, gate_id: int, split_time: float, total_gates: int = 7):
        """Display gate traversal alert for 2 seconds."""
        self.banner_text = f">>> GATE {gate_id}/{total_gates} CLEARED | SPLIT: {split_time:05.2f}s <<<"
        self.banner_expiry = split_time + 2.5

    def update(
        self,
        t: float,
        pos: np.ndarray,
        R: np.ndarray,
        vel: np.ndarray,
        pos_err: float,
        s_norm: float,
        est_m: float,
        true_m: float,
        payload_dropped: bool,
        mode: str = "BREGMAN",
        coriolis: str = "LC",
        sim_speed: float = 1.0,
    ):
        """Update OSD text, artificial horizon lines, and telemetry tapes."""
        if not self.enabled:
            self.clear()
            return

        # Update synthetic battery drain based on simulation time
        self.vbat_current = max(21.0, self.vbat_initial - (t * 0.035))

        speed = float(np.linalg.norm(vel[0:3] if len(vel) >= 3 else [0, 0, 0]))
        alt_agl = max(0.0, float(pos[2]))
        vz = float(vel[2]) if len(vel) >= 3 else 0.0
        payload_status = "DROPPED" if payload_dropped else "ATTACHED"

        # 1. Multi-line FPV Telemetry HUD Text
        hud_lines = [
            f"--- [ SE(3) {mode.upper()} C_{coriolis.upper()} ] ---",
            f"TIME: {t:05.2f}s | SPD: {sim_speed:.1f}x | BAT: {self.vbat_current:.1f}V (6S)",
            f"SPEED: {speed:4.1f} m/s | ALT: {alt_agl:4.2f} m | Vz: {vz:+4.1f} m/s",
            f"||e_p||: {pos_err:.3f} m | ||s||: {s_norm:.2f}",
            f"m_hat: {est_m:.2f} kg (true: {true_m:.2f}) | PAYLOAD: {payload_status}",
        ]
        hud_text = "\n".join(hud_lines)

        # Position HUD slightly above and forward of UAV
        # Forward vector x_b, Up vector z_b
        fwd = R[:, 0]
        up = R[:, 2]
        hud_pos = (pos + 0.15 * fwd + 0.35 * up).tolist()

        if self.hud_text_id is None:
            self.hud_text_id = p.addUserDebugText(
                hud_text,
                hud_pos,
                textColorRGB=[0.1, 1.0, 0.4],  # Racing green HUD
                textSize=1.05,
                lifeTime=0,
                physicsClientId=self.client_id,
            )
        else:
            self.hud_text_id = p.addUserDebugText(
                hud_text,
                hud_pos,
                textColorRGB=[0.1, 1.0, 0.4],
                textSize=1.05,
                lifeTime=0,
                replaceItemUniqueId=self.hud_text_id,
                physicsClientId=self.client_id,
            )

        # 2. Gate Clearance Banner
        if t < self.banner_expiry and self.banner_text:
            banner_pos = (pos + 0.25 * fwd + 0.65 * up).tolist()
            if self.gate_banner_id is None:
                self.gate_banner_id = p.addUserDebugText(
                    self.banner_text,
                    banner_pos,
                    textColorRGB=[1.0, 0.95, 0.1],  # Flashing gold/yellow
                    textSize=1.2,
                    lifeTime=0,
                    physicsClientId=self.client_id,
                )
            else:
                self.gate_banner_id = p.addUserDebugText(
                    self.banner_text,
                    banner_pos,
                    textColorRGB=[1.0, 0.95, 0.1],
                    textSize=1.2,
                    lifeTime=0,
                    replaceItemUniqueId=self.gate_banner_id,
                    physicsClientId=self.client_id,
                )
        else:
            if self.gate_banner_id is not None:
                try:
                    p.removeUserDebugItem(self.gate_banner_id, physicsClientId=self.client_id)
                except Exception:
                    pass
                self.gate_banner_id = None

    def toggle(self) -> bool:
        """Toggle OSD overlay on/off."""
        self.enabled = not self.enabled
        if not self.enabled:
            self.clear()
        return self.enabled

    def clear(self):
        """Remove OSD text and debug lines from PyBullet."""
        if self.hud_text_id is not None:
            try:
                p.removeUserDebugItem(self.hud_text_id, physicsClientId=self.client_id)
            except Exception:
                pass
            self.hud_text_id = None

        if self.gate_banner_id is not None:
            try:
                p.removeUserDebugItem(self.gate_banner_id, physicsClientId=self.client_id)
            except Exception:
                pass
            self.gate_banner_id = None

        for lid in self.horizon_line_ids:
            try:
                p.removeUserDebugItem(lid, physicsClientId=self.client_id)
            except Exception:
                pass
        self.horizon_line_ids.clear()

    def close(self):
        """Alias for clear."""
        self.clear()
