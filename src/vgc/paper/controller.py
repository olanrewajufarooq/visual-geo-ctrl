"""Nominal geometric tracking controller on SE(3)."""

from dataclasses import dataclass

import numpy as np

from ..math.inertia import inertia_from_pi
from ..math.se3 import adjoint_se3, ad_twist, inv_se3, skew
from .errors import potential, potential_derivative
from .regressor import regressor


@dataclass
class ControllerDiagnostics:
    He: np.ndarray
    Ve: np.ndarray
    eH: np.ndarray
    eHdot: np.ndarray
    Psi: float
    Vr: np.ndarray
    VrDot: np.ndarray
    s: np.ndarray
    D: np.ndarray
    Wg: np.ndarray
    Y: np.ndarray
    Vs: float
    sNorm: float


def controller(state: dict, desired: dict, cfg: dict, estimate: np.ndarray | None = None):
    """Evaluate the known-inertia nominal body-wrench controller."""
    H = np.asarray(state["H"], dtype=float)
    V = np.asarray(state["V"], dtype=float).ravel()
    Hd = np.asarray(desired["H"], dtype=float)
    Vd = np.asarray(desired["V"], dtype=float).ravel()
    Vd_dot = np.asarray(desired.get("Vdot", np.zeros(6)), dtype=float).ravel()

    He = inv_se3(Hd) @ H
    KR = np.asarray(cfg["KR"], dtype=float)
    Kxi = np.asarray(cfg["Kxi"], dtype=float)
    eH, psi = potential(He, KR, Kxi)
    He_inv = inv_se3(He)
    transported_vd = adjoint_se3(He_inv) @ Vd
    Ve = V - transported_vd
    Lambda = np.asarray(cfg["Lambda"], dtype=float)
    Vr = transported_vd - Lambda @ eH
    eHdot = potential_derivative(He, Ve, KR, Kxi)
    Vr_dot = -ad_twist(Ve) @ transported_vd + adjoint_se3(He_inv) @ Vd_dot - Lambda @ eHdot

    pi = np.asarray(cfg["plantPi"], dtype=float).ravel()
    gravity = np.asarray(cfg["gravity"], dtype=float).ravel()
    I6 = inertia_from_pi(pi)
    g_body = H[0:3, 0:3].T @ gravity
    Wg = np.concatenate([skew(pi[1:4]) @ g_body, pi[0] * g_body])
    s = V - Vr
    Lambda_s = np.asarray(cfg.get("Lambda_s", np.linalg.inv(Lambda)), dtype=float)
    Lambda_s_s = Lambda_s @ s
    s_norm_sq = max(0.0, float(s @ Lambda_s_s))
    s_norm = np.sqrt(s_norm_sq)
    alpha = float(cfg["alpha"])
    kd = float(cfg["kd"])
    ks = float(cfg["ks"])
    D = np.zeros(6) if s_norm == 0.0 else (kd + ks * s_norm ** (alpha - 1.0)) * Lambda_s_s

    Y = regressor(H, V, Vr, Vr_dot, gravity, str(cfg["coriolis"]).lower())
    W = Y @ pi - D
    diagnostics = ControllerDiagnostics(
        He=He, Ve=Ve, eH=eH, eHdot=eHdot, Psi=psi, Vr=Vr, VrDot=Vr_dot,
        s=s, D=D, Wg=Wg, Y=Y, Vs=float(0.5 * (s @ (I6 @ s))), sNorm=s_norm,
    )
    return W, diagnostics
