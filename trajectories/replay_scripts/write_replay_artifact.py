"""Save processed replay trajectory artifacts."""

import json
from pathlib import Path
from typing import Dict, Any, Union
import numpy as np


def write_replay_artifact(
    output_path: Union[str, Path],
    traj: Dict[str, Any],
    output_format: str = "npz",
) -> Path:
    """Save one processed replay trajectory to disk in .npz (or .mat) format.

    Parameters
    ----------
    output_path : str or Path
        Target destination file path.
    traj : dict
        Processed trajectory dictionary containing t, p, v_b, a_b, omega_b, alpha_b, R, meta.
    output_format : str, optional
        Target format, either 'npz' (default) or 'mat'.

    Returns
    -------
    Path
        Resolved path where artifact was saved.
    """
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    fmt = output_format.lower()
    if output_path.suffix == ".mat" or fmt == "mat":
        import scipy.io as sio

        if output_path.suffix != ".mat":
            output_path = output_path.with_suffix(".mat")
        sio.savemat(str(output_path), {"traj": traj})
        return output_path

    # Default to .npz
    if output_path.suffix != ".npz":
        output_path = output_path.with_suffix(".npz")

    meta = traj.get("meta", {})
    if not isinstance(meta, str):
        meta_str = json.dumps(meta, default=str)
    else:
        meta_str = meta

    np.savez_compressed(
        str(output_path),
        t=np.asarray(traj["t"], dtype=float).ravel(),
        p=np.asarray(traj["p"], dtype=float),
        v_b=np.asarray(traj["v_b"], dtype=float),
        a_b=np.asarray(traj["a_b"], dtype=float),
        omega_b=np.asarray(traj["omega_b"], dtype=float),
        alpha_b=np.asarray(traj["alpha_b"], dtype=float),
        R=np.asarray(traj["R"], dtype=float),
        meta=meta_str,
    )
    return output_path
