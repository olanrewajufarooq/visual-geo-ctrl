"""Encoding, decoding, block updating, and precision rounding for optimizer candidates."""

import copy
from typing import Dict, Any
import numpy as np

from .bounds import gain_block_indices


def round_significant(value: Any, sig_figs: int = 4) -> Any:
    """Round scalar or array to a fixed number of significant figures."""
    val = np.asarray(value, dtype=float)
    is_scalar = val.ndim == 0
    val_flat = val.ravel()

    result = np.zeros_like(val_flat)
    nonzero = val_flat != 0
    if np.any(nonzero):
        scale = 10.0 ** (sig_figs - 1 - np.floor(np.log10(np.abs(val_flat[nonzero]))))
        result[nonzero] = np.round(val_flat[nonzero] * scale) / scale

    if is_scalar:
        return float(result[0])
    return result.reshape(val.shape)


def round_gains(gains: Dict[str, Any], sig_figs: int = 4) -> Dict[str, Any]:
    """Round all gain fields to a fixed number of significant figures."""
    fields = ["KRdiag", "Kxidiag", "LambdaDiag", "kd", "ks", "alpha", "gammaE", "gammaB"]
    rounded = {}
    for f in fields:
        if f in gains and gains[f] is not None:
            rounded[f] = round_significant(gains[f], sig_figs)
        else:
            rounded[f] = gains.get(f, None)
    return rounded


def encode_scenario_gains(scenario: Dict[str, Any]) -> np.ndarray:
    """Encode a scenario controller dictionary into an optimizer candidate vector."""
    if "controller" not in scenario:
        raise ValueError("scenario must contain a 'controller' dict.")
    c = scenario["controller"]
    mode = str(c["mode"]).lower()

    kr = np.diag(c["KR"]) if c["KR"].ndim == 2 else np.asarray(c["KR"], dtype=float).ravel()
    kxi = np.diag(c["Kxi"]) if c["Kxi"].ndim == 2 else np.asarray(c["Kxi"], dtype=float).ravel()
    lam = np.diag(c["Lambda"]) if c["Lambda"].ndim == 2 else np.asarray(c["Lambda"], dtype=float).ravel()
    kd = float(c["kd"])
    ks = float(c["ks"])
    alpha = float(c["alpha"])

    positive = np.concatenate([kr, kxi, lam, [kd, ks]])
    if np.any(positive <= 0):
        raise ValueError("All stiffness, sliding, and damping gains must be strictly positive.")
    if not (0.0 < alpha < 1.0):
        raise ValueError(f"alpha must be in (0, 1), got {alpha}")

    candidate = list(np.log10(positive)) + [alpha]

    if mode == "euclidean":
        ge = np.asarray(c["gammaE"], dtype=float).ravel()
        if len(ge) != 10 or np.any(ge <= 0):
            raise ValueError("gammaE must have 10 strictly positive values.")
        candidate.extend(np.log10(ge).tolist())
    elif mode == "bregman":
        gb = float(c["gammaB"])
        if gb <= 0:
            raise ValueError("gammaB must be strictly positive.")
        candidate.append(np.log10(gb))
    elif mode != "nominal":
        raise ValueError(f"Unsupported controller mode: {mode}")

    return np.array(candidate, dtype=float)


def encode_adaptive_base_gains(
    tracking_candidate: np.ndarray, gamma_e: np.ndarray, gamma_b: float,
) -> np.ndarray:
    """Encode shared tracking gains with Euclidean and Bregman rates."""
    tracking = np.asarray(tracking_candidate, dtype=float).ravel()
    euclidean = np.asarray(gamma_e, dtype=float).ravel()
    bregman = float(gamma_b)
    if tracking.size != 15 or not np.all(np.isfinite(tracking)):
        raise ValueError("Adaptive-base tracking candidate must contain 15 finite coordinates.")
    if euclidean.size != 10 or not np.all(np.isfinite(euclidean)) or np.any(euclidean <= 0.0):
        raise ValueError("gammaE must contain 10 finite, strictly positive values.")
    if not np.isfinite(bregman) or bregman <= 0.0:
        raise ValueError("gammaB must be finite and strictly positive.")
    return np.concatenate([tracking, np.log10(euclidean), [np.log10(bregman)]])


def apply_scenario_gains(candidate: np.ndarray, scenario: Dict[str, Any]) -> Dict[str, Any]:
    """Decode candidate vector and apply to a deep copy of scenario."""
    sc = copy.deepcopy(scenario)
    c = sc["controller"]
    mode = str(c["mode"]).lower()

    expected_len = 15 if mode == "nominal" else (16 if mode == "bregman" else 25)
    cand = np.asarray(candidate, dtype=float).ravel()
    if len(cand) != expected_len:
        raise ValueError(f"Candidate length {len(cand)} does not match expected {expected_len} for mode {mode}")

    old_lambda = np.asarray(c["Lambda"], dtype=float)
    old_lambda_s = np.asarray(c.get("Lambda_s", np.linalg.inv(old_lambda)), dtype=float)
    lambda_s_tracks_lambda = np.allclose(old_lambda_s, np.linalg.inv(old_lambda))

    positive = 10.0 ** cand[0:14]
    alpha = float(cand[14])

    if not (0.0 < alpha < 1.0):
        raise ValueError(f"alpha must be strictly in (0, 1), got {alpha}")
    if positive[12] <= 0.5:
        raise ValueError(f"kd must be strictly > 0.5, got {positive[12]}")

    c["KR"] = np.diag(positive[0:3])
    c["Kxi"] = np.diag(positive[3:6])
    c["Lambda"] = np.diag(positive[6:12])
    if lambda_s_tracks_lambda:
        c["Lambda_s"] = np.linalg.inv(c["Lambda"])
    c["kd"] = float(positive[12])
    c["ks"] = float(positive[13])
    c["alpha"] = alpha

    if mode == "euclidean":
        c["gammaE"] = 10.0 ** cand[15:25]
    elif mode == "bregman":
        c["gammaB"] = float(10.0 ** cand[15])

    sc["controller"] = c
    return sc


def apply_gain_block(
    block_candidate: np.ndarray,
    incumbent_candidate: np.ndarray,
    block: str,
    scenario: Dict[str, Any],
) -> Dict[str, Any]:
    """Decode a sub-block candidate while retaining all fixed incumbent parameters."""
    mode = str(scenario["controller"]["mode"]).lower()
    indices = gain_block_indices(mode, block)

    cand = np.copy(incumbent_candidate).ravel()
    b_cand = np.asarray(block_candidate, dtype=float).ravel()
    if len(b_cand) != len(indices):
        raise ValueError(f"Block candidate length {len(b_cand)} does not match indices {len(indices)} for block {block}")

    cand[indices] = b_cand
    return apply_scenario_gains(cand, scenario)
