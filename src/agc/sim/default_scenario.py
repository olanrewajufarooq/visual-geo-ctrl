"""Paper-aligned default simulation scenario builder."""

import os
from pathlib import Path
from typing import Optional
import numpy as np

from ..math.inertia import pseudo_from_pi
from ..plant.compound_pi import compound_pi
from .replay_trajectory import ReplayTrajectory


PAYLOAD_PROFILES = {
    "evaluation": {
        "mass": 0.75,
        "dimensions": np.array([0.12, 0.12, 0.08]),
        "center": np.array([0.20, 0.05, -0.12]),
    },
    "flat_light": {
        "mass": 0.60,
        "dimensions": np.array([0.16, 0.10, 0.06]),
        "center": np.array([0.20, 0.05, -0.12]),
    },
    "tall_heavy": {
        "mass": 0.90,
        "dimensions": np.array([0.10, 0.10, 0.16]),
        "center": np.array([0.20, 0.05, -0.12]),
    },
}


def get_payload_profile(name: str) -> dict:
    """Return an independent payload definition for a named benchmark profile."""
    key = str(name).lower()
    if key not in PAYLOAD_PROFILES:
        choices = ", ".join(PAYLOAD_PROFILES)
        raise ValueError(f"Unknown payload profile {name!r}; expected one of: {choices}.")
    profile = PAYLOAD_PROFILES[key]
    return {
        "mass": float(profile["mass"]),
        "dimensions": np.array(profile["dimensions"], dtype=float),
        "center": np.array(profile["center"], dtype=float),
    }


def get_repository_root() -> Path:
    # 3 levels up from src/agc/sim/default_scenario.py is repo root
    return Path(__file__).resolve().parent.parent.parent.parent


def default_scenario(
    replay_id: str = "lemniscate_01_auto",
    mode: str = "bregman",
    coriolis: str = "lc",
    duration: float = 30.0,
    gain_source: str = "optimized",
    gui: bool = False,
    sim_speed: float = 1.0,
    enable_pacing: bool = True,
    ground_z: float = 0.0,
    ground_style: str = "arena",
    gates_mode: Optional[str] = None,
    cam_mode: str = "chase",
    enable_osd: bool = False,
    drone_type: str = "pybullet_drones",
    initial_offset: Optional[np.ndarray] = None,
    payload_profile: str = "evaluation",
) -> dict:
    """Build paper-validation benchmark scenario with 10s payload drop."""
    root = get_repository_root()
    processed_dir = root / "trajectories" / "processed"
    npz_file = processed_dir / f"{replay_id}.npz"
    traj_file = npz_file if npz_file.is_file() else (processed_dir / f"{replay_id}.mat")
    sampler = ReplayTrajectory(str(traj_file))
    desired0 = sampler.sample(0.0)

    # UAV physical parameters
    m = 3.646
    cog = np.array([0.0, 0.0, -0.00229])
    I = np.array([0.04092, 0.04017, 0.06921, 5.656e-5, 1.313e-5, -6.494e-5])
    pi = np.concatenate([[m], m * cog, I])

    # Named cuboid payload profile; its compound inertia is derived below.
    selected_payload_profile = str(payload_profile).lower()
    payload = get_payload_profile(selected_payload_profile)
    loaded_pi = compound_pi(pi, payload)
    payload_drop = {
        "releaseTime": 10.0,
        "barePi": pi,
        "loadedPi": loaded_pi,
        "payload": payload,
    }

    # Initial condition (starts directly on the ground / launch pad by default)
    initial_H = np.copy(desired0["H"])
    if initial_offset is not None:
        initial_H[0:3, 3] += np.asarray(initial_offset, dtype=float)
    initial_state = {"H": initial_H, "V": np.copy(desired0["V"])}

    # Gain selection
    mode_lower = mode.lower()
    coriolis_lower = coriolis.lower()
    if gain_source.lower() == "manual":
        from ..config.manual_gains import manual_gains
        gains = manual_gains(mode_lower, coriolis_lower)
    elif gain_source.lower() == "optimized":
        from ..config.optimized_gains import optimized_gains
        gains = optimized_gains(mode_lower, coriolis_lower)
    else:
        raise ValueError(f"Unknown gain_source: '{gain_source}'.")

    controller_cfg = {
        "mode": mode_lower,
        "coriolis": coriolis_lower,
        "KR": np.diag(gains["KRdiag"]),
        "Kxi": np.diag(gains["Kxidiag"]),
        "Lambda": np.diag(gains["LambdaDiag"]),
        "kd": float(gains["kd"]),
        "ks": float(gains["ks"]),
        "alpha": float(gains["alpha"]),
        "gravity": np.array([0.0, 0.0, 9.81]),
        "gammaE": np.asarray(gains["gammaE"], dtype=float),
        "gammaB": float(gains["gammaB"]),
    }

    initial_estimate = np.copy(loaded_pi)
    if mode_lower == "bregman":
        initial_estimate = pseudo_from_pi(initial_estimate)

    if gates_mode is None:
        if "lemniscate" in replay_id.lower():
            gates_mode = "lemniscate"
        elif "ratm" in replay_id.lower():
            gates_mode = "ratm"
        else:
            gates_mode = "auto"

    return {
        "plantPi": pi,
        "initial": initial_state,
        "trajectory": sampler.sample,
        "duration": float(duration),
        "dtPlant": 0.002,
        "dtControl": 0.02,
        "dtAdaptation": 0.01,
        "plantGravity": np.array([0.0, 0.0, -9.81]),
        "controller": controller_cfg,
        "initialEstimate": initial_estimate,
        "payloadDrop": payload_drop,
        "gui": gui,
        "simSpeed": sim_speed,
        "enablePacing": enable_pacing,
        "replayId": replay_id,
        "groundZ": ground_z,
        "groundStyle": ground_style,
        "gatesMode": gates_mode,
        "camMode": cam_mode,
        "enableOsd": enable_osd,
        "droneType": drone_type,
        "payloadProfile": selected_payload_profile,
    }
