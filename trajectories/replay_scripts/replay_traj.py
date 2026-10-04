"""Replay a processed trajectory artifact as a reference state generator."""

from pathlib import Path
from typing import Dict, Any, Tuple, Union
import numpy as np

from vgc.sim.replay_trajectory import ReplayTrajectory
from .replay_processor import ReplayProcessor


class ReplayTraj:
    """Replay trajectory reference matching MATLAB ReplayTraj."""

    def __init__(self, cfg_or_path: Union[str, Path, Dict[str, Any]]):
        if isinstance(cfg_or_path, (str, Path)):
            path_str = str(cfg_or_path)
            # If given a simple ID or full path
            if not Path(path_str).is_file():
                entry = ReplayProcessor.load_entry(path_str)
                path_str = entry["artifact_path"]
            self.sampler = ReplayTrajectory(path_str)
        elif isinstance(cfg_or_path, dict):
            entry = ReplayProcessor.load_entry(cfg_or_path)
            self.sampler = ReplayTrajectory(entry["artifact_path"])
        else:
            raise TypeError(f"Invalid trajectory specification: {type(cfg_or_path)}")

        self.data = self.sampler.traj

    def generate(self, t: float, *args) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Sample H (4x4), V (6,), A (6,) at time t."""
        sample = self.sampler.sample(t)
        return sample["H"], sample["V"], sample["Vdot"]

    def reset(self, *args) -> None:
        """Replay trajectory is stateless after construction."""
        pass

    def __call__(self, t: float) -> Dict[str, np.ndarray]:
        return self.sampler.sample(t)
