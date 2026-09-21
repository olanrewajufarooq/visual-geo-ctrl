"""Paper-aligned default simulation scenario builder."""

import os
from pathlib import Path
import numpy as np

from ..math.inertia import pseudo_from_pi
from ..plant.compound_pi import compound_pi
from .replay_trajectory import ReplayTrajectory


def get_repository_root() -> Path:
    # 3 levels up from src/agc/sim/default_scenario.py is repo root
    return Path(__file__).resolve().parent.parent.parent.parent


def default_scenario(
    replay_id: str = "lemniscate_01_auto",
    mode: str = "bregman",
    coriolis: str = "c1",
    duration: float = 30.0,
    gain_source: str = "optimized",
    gui: bool = False,
    sim_speed: float = 1.0,
    enable_pacing: bool = True,
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

    # Asymmetric cuboid payload
    payload = {
        "mass": 0.75,
        "dimensions": np.array([0.12, 0.12, 0.08]),
        "center": np.array([0.20, 0.05, -0.12]),
    }
    loaded_pi = compound_pi(pi, payload)
    payload_drop = {
        "releaseTime": 10.0,
        "barePi": pi,
        "loadedPi": loaded_pi,
        "payload": payload,
    }

    # Initial condition with offset
    initial_H = np.copy(desired0["H"])
    initial_H[0:3, 3] += np.array([0.2, -0.1, 0.15])
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
    }
