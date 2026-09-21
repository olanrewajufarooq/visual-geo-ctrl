"""Persistence utilities for saving and loading AGC simulation runs."""

import os
from pathlib import Path
from typing import Dict, Any
import numpy as np


def save_run(save_dir: str, run: Dict[str, Any], metrics: Dict[str, float], scenario: dict):
    """Save simulation run data and metrics to directory."""
    os.makedirs(save_dir, exist_ok=True)
    npz_path = os.path.join(save_dir, "run.npz")

    # Filter arrays for npz
    arrays = {}
    for key, val in run.items():
        if isinstance(val, np.ndarray):
            arrays[key] = val
        elif isinstance(val, (int, float, str, bool)):
            arrays[key] = np.array(val)

    np.savez_compressed(npz_path, **arrays)

    # Save metrics text
    metrics_path = os.path.join(save_dir, "metrics.txt")
    with open(metrics_path, "w", encoding="utf-8") as f:
        for k, v in metrics.items():
            f.write(f"{k}: {v}\n")


def load_run(save_dir: str) -> Dict[str, Any]:
    """Load simulation run data from directory."""
    npz_path = os.path.join(save_dir, "run.npz")
    if not os.path.isfile(npz_path):
        raise FileNotFoundError(f"Run file not found: {npz_path}")

    with np.load(npz_path, allow_pickle=True) as data:
        run = {key: data[key] for key in data.files}
    return run
