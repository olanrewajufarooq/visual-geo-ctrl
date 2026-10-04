"""Nominal simulation scenario construction."""

from pathlib import Path
from typing import Optional

import numpy as np

from .replay_trajectory import ReplayTrajectory


def get_repository_root() -> Path:
    return Path(__file__).resolve().parent.parent.parent.parent


def default_scenario(
    replay_id: str = "lemniscate_01_auto",
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
) -> dict:
    """Build a bare-vehicle nominal tracking scenario."""
    root = get_repository_root()
    processed_dir = root / "trajectories" / "processed"
    npz_file = processed_dir / f"{replay_id}.npz"
    traj_file = npz_file if npz_file.is_file() else (processed_dir / f"{replay_id}.mat")
    sampler = ReplayTrajectory(str(traj_file))
    desired0 = sampler.sample(0.0)

    mass = 3.646
    cog = np.array([0.0, 0.0, -0.00229])
    inertia = np.array([0.04092, 0.04017, 0.06921, 5.656e-5, 1.313e-5, -6.494e-5])
    plant_pi = np.concatenate([[mass], mass * cog, inertia])

    initial_H = np.copy(desired0["H"])
    if initial_offset is not None:
        initial_H[0:3, 3] += np.asarray(initial_offset, dtype=float)

    coriolis_lower = str(coriolis).lower()
    if gain_source.lower() == "manual":
        from ..config.manual_gains import manual_gains
        gains = manual_gains("nominal", coriolis_lower)
    elif gain_source.lower() == "optimized":
        from ..config.optimized_gains import optimized_gains
        gains = optimized_gains("nominal", coriolis_lower)
    else:
        raise ValueError(f"Unknown gain_source: {gain_source!r}.")

    controller_cfg = {
        "mode": "nominal",
        "coriolis": coriolis_lower,
        "plantPi": plant_pi,
        "KR": np.diag(gains["KRdiag"]),
        "Kxi": np.diag(gains["Kxidiag"]),
        "Lambda": np.diag(gains["LambdaDiag"]),
        "Lambda_s": np.diag(1.0 / np.asarray(gains["LambdaDiag"], dtype=float)),
        "kd": float(gains["kd"]),
        "ks": float(gains["ks"]),
        "alpha": float(gains["alpha"]),
        "gravity": np.array([0.0, 0.0, 9.81]),
    }

    if gates_mode is None:
        if "lemniscate" in replay_id.lower():
            gates_mode = "lemniscate"
        elif "ratm" in replay_id.lower():
            gates_mode = "ratm"
        else:
            gates_mode = "auto"

    return {
        "plantPi": plant_pi,
        "initial": {"H": initial_H, "V": np.copy(desired0["V"])},
        "trajectory": sampler.sample,
        "duration": float(duration),
        "dtPlant": 0.002,
        "dtControl": 0.02,
        "plantGravity": np.array([0.0, 0.0, -9.81]),
        "controller": controller_cfg,
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
    }
