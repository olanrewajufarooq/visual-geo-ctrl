"""Diagnostics for paper-level controller identities."""

from typing import Any, Dict
import numpy as np

from .controller import controller
from .coriolis import coriolis
from ..math.inertia import inertia_from_pi


def connection_identity(
    state: Dict[str, Any],
    desired: Dict[str, Any],
    controller_cfg: Dict[str, Any],
    true_pi: np.ndarray,
) -> Dict[str, np.ndarray | float]:
    """Evaluate the LC/RB wrench identity for one controller state."""
    pi = np.asarray(true_pi, dtype=float).ravel()
    cfg_lc = dict(controller_cfg, mode="nominal", coriolis="lc")
    cfg_rb = dict(controller_cfg, mode="nominal", coriolis="rb")
    wrench_lc, diagnostics, _ = controller(state, desired, cfg_lc, pi)
    wrench_rb, _, _ = controller(state, desired, cfg_rb, pi)
    I6 = inertia_from_pi(pi)
    theoretical = -(
        coriolis("rb", state["V"], I6, diagnostics.Vr)
        - coriolis("lc", state["V"], I6, diagnostics.Vr)
    )
    difference = wrench_rb - wrench_lc
    residual = difference - theoretical
    return {
        "wrenchDifference": difference,
        "theoreticalTerm": theoretical,
        "residual": residual,
        "differenceNorm": float(np.linalg.norm(difference)),
        "theoreticalNorm": float(np.linalg.norm(theoretical)),
        "residualNorm": float(np.linalg.norm(residual)),
    }
