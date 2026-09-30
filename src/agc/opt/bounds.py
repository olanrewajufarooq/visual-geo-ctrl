"""Gain bounds, parameter block indices, and staged optimization schedules."""

from typing import Tuple, List, Union
import numpy as np


def gain_bounds(mode: str) -> Tuple[np.ndarray, np.ndarray]:
    """Default all-gain optimizer bounds in log10 coordinates for positive gains and linear for alpha."""
    mode = mode.lower()
    if mode not in ("nominal", "adaptive_base", "euclidean", "bregman"):
        raise ValueError(f"Unknown controller mode: {mode}")

    # 14 positive parameters in log10 space:
    # 3 KR, 3 Kxi, 3 Lambda_R, 3 Lambda_p, 1 kd (> 0.5), 1 ks
    pos_lb = [
        np.log10(1e-3), np.log10(1e-3), np.log10(1e-3),   # KRdiag
        np.log10(1e-2), np.log10(1e-2), np.log10(1e-2),   # Kxidiag
        np.log10(1e-2), np.log10(1e-2), np.log10(1e-2),   # LambdaDiag (angular)
        np.log10(1e-2), np.log10(1e-2), np.log10(1e-2),   # LambdaDiag (linear)
        np.log10(0.55),                                    # kd (kd > 0.5)
        np.log10(1e-3),                                    # ks
    ]
    pos_ub = [
        np.log10(30.0), np.log10(30.0), np.log10(30.0),   # KRdiag
        np.log10(300.0), np.log10(300.0), np.log10(300.0), # Kxidiag
        np.log10(100.0), np.log10(100.0), np.log10(100.0), # LambdaDiag (angular)
        np.log10(10.0), np.log10(10.0), np.log10(10.0),   # LambdaDiag (linear)
        np.log10(100.0),                                   # kd
        np.log10(50.0),                                    # ks
    ]

    # Coordinate 15: alpha in physical coordinates [0.30, 0.95].
    # Lower bound raised from 0.05: near-zero alpha makes ks*||s||^(alpha-1)
    # diverge as ||s|| -> 0, causing sign-flipping torque chattering every
    # control tick.  alpha >= 0.30 keeps the power-law finite.
    lb = pos_lb + [0.30]
    ub = pos_ub + [0.95]

    if mode in ("euclidean", "adaptive", "adaptive_base"):
        # 10 adaptation gains in log10 space [-5, 0]
        lb.extend([np.log10(1e-5)] * 10)
        ub.extend([0.0] * 10)
    if mode in ("bregman", "adaptive_base"):
        # 1 scalar adaptation gain gammaB in log10 space [-5, -1] (1e-5 to 1e-1)
        lb.append(np.log10(1e-5))
        ub.append(np.log10(1e-1))

    return np.array(lb, dtype=float), np.array(ub, dtype=float)


def gain_block_indices(mode: str, block: str) -> List[int]:
    """Return optimizer-coordinate indices (0-indexed) for a given gain block."""
    mode = mode.lower()
    block = block.lower()

    if mode == "nominal":
        total = 15
    elif mode == "adaptive_base":
        total = 26
    elif mode == "bregman":
        total = 16
    elif mode in ("euclidean", "adaptive"):
        total = 25
    else:
        raise ValueError(f"Unknown controller mode: {mode}")

    if block == "all":
        return list(range(total))
    elif block == "nonadaptive":
        return list(range(15))
    elif block == "tracking":
        # KR (0..2), Kxi (3..5)
        return list(range(0, 6))
    elif block == "sliding_dissipation":
        # Lambda (6..11), kd (12), ks (13), alpha (14)
        return list(range(6, 15))
    elif block in ("sliding_metric", "lambda"):
        # Lambda (6..11)
        return list(range(6, 12))
    elif block in ("dissipation", "damping"):
        # kd (12), ks (13), alpha (14)
        return list(range(12, 15))
    elif block == "adaptive":
        # gammaB (15), gammaE (15..24), or adaptive_base's gammaE (15..24) + gammaB (25)
        return list(range(15, total))
    else:
        raise ValueError(f"Unknown gain block: {block}")


def gain_optimization_stages(mode: str, schedule: str = "hierarchical") -> List[str]:
    """Return the ordered staged optimization schedule."""
    mode = mode.lower()
    schedule = schedule.lower()

    if mode in ("nominal", "adaptive_base"):
        if schedule == "classic":
            return ["all"]
        stages = [
            "all",
            "tracking",
            "sliding_dissipation",
            "sliding_metric",
            "dissipation",
        ]
        if mode == "adaptive_base":
            stages.append("adaptive")
        stages.append("all")
        return stages
    elif mode in ("adaptive", "euclidean", "bregman"):
        if schedule == "classic":
            return ["all", "nonadaptive", "adaptive"]
        return [
            "all",
            "tracking",
            "sliding_dissipation",
            "sliding_metric",
            "dissipation",
            "adaptive",
        ]
    else:
        raise ValueError(f"Unknown controller mode: {mode}. Choose 'nominal', 'adaptive_base', or 'adaptive'.")


def expand_scenario_selection(
    modes: Union[str, List[str], None] = None,
    coriolis: Union[str, List[str], None] = None,
) -> List[str]:
    """Expand mode selector into unique optimization modes ('nominal', 'adaptive')."""
    available_modes = ["nominal", "adaptive"]

    if not modes or (isinstance(modes, str) and modes.lower() == "all"):
        return list(available_modes)
    elif isinstance(modes, str):
        sel_modes = [modes.lower()]
    else:
        sel_modes = [m.lower() for m in modes]

    for m in sel_modes:
        if m not in available_modes:
            raise ValueError(f"Unsupported mode selector: {m}. Choose 'nominal' or 'adaptive'.")

    return sel_modes
