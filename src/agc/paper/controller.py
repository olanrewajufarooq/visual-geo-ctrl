"""Paper-aligned SE(3) tracking controller with geometric adaptation."""

from dataclasses import dataclass
from typing import Optional, Tuple, Any
import numpy as np

from ..math.se3 import inv_se3, adjoint_se3, ad_twist, skew
from ..math.inertia import inertia_from_pi, pi_from_pseudo
from .errors import potential, potential_derivative
from .regressor import regressor
from .adaptation import euclidean_step, bregman_step, pseudo_gradient


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


def parameter_state(mode: str, estimate: Any) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """Convert mode-specific estimate to 10-parameter piHat and 4x4 Jhat."""
    mode_lower = mode.lower()
    if mode_lower in ("nominal", "euclidean"):
        pi_hat = np.asarray(estimate, dtype=float).ravel()
        return pi_hat, None
    elif mode_lower == "bregman":
        J_hat = np.asarray(estimate, dtype=float)
        pi_hat = pi_from_pseudo(J_hat)
        return pi_hat, J_hat
    else:
        raise ValueError(f"Unknown controller mode: '{mode}'.")


def controller(
    state: dict,
    desired: dict,
    cfg: dict,
    estimate: Any,
    dt_adapt: Optional[float] = None,
) -> Tuple[np.ndarray, ControllerDiagnostics, Any]:
    """Evaluate paper-aligned body-wrench tracking controller and parameter update.

    Parameters:
    -----------
    state : dict
        'H': (4, 4) pose, 'V': (6,) left-trivialized body twist [omega; v].
    desired : dict
        'H': (4, 4) desired pose, 'V': (6,) desired twist, 'Vdot': (6,) desired acceleration.
    cfg : dict
        Controller configuration with KR, Kxi, Lambda, kd, ks, alpha, gravity, mode, coriolis,
        gammaE, gammaB.
    estimate : array-like
        Current estimator state: pi (10,) for nominal/euclidean, or J (4, 4) for bregman.
    dt_adapt : float or None
        Adaptation timestep. If None, no parameter update is taken.

    Returns:
    --------
    W : (6,) ndarray
        Commanded body wrench [tau_b; f_b].
    diagnostics : ControllerDiagnostics
        Detailed diagnostic signals.
    estimate_next : array-like
        Updated estimator state.
    """
    H = np.asarray(state['H'], dtype=float)
    V = np.asarray(state['V'], dtype=float).ravel()
    Hd = np.asarray(desired['H'], dtype=float)
    Vd = np.asarray(desired['V'], dtype=float).ravel()
    VdDot = np.asarray(desired.get('Vdot', np.zeros(6)), dtype=float).ravel()

    # Configuration error: He = Hd^{-1} * H
    He = inv_se3(Hd) @ H
    KR = np.asarray(cfg['KR'], dtype=float)
    Kxi = np.asarray(cfg['Kxi'], dtype=float)
    eH, psi = potential(He, KR, Kxi)

    # Transport desired body twist from Hd to the current error frame
    He_inv = inv_se3(He)
    transportedVd = adjoint_se3(He_inv) @ Vd
    Ve = V - transportedVd

    # Sliding variable and reference body velocity
    Lambda = np.asarray(cfg['Lambda'], dtype=float)
    if Lambda.ndim == 1:
        Lambda = np.diag(Lambda)
    Vr = transportedVd - Lambda @ eH

    # Differentiate Vr analytically along error kinematics
    eHdot = potential_derivative(He, Ve, KR, Kxi)
    VrDot = -ad_twist(Ve) @ transportedVd + adjoint_se3(He_inv) @ VdDot - Lambda @ eHdot

    # Estimated inertial model
    mode = str(cfg['mode']).lower()
    coriolis_form = str(cfg['coriolis']).lower()
    gravity = np.asarray(cfg['gravity'], dtype=float).ravel()

    piHat, Jhat = parameter_state(mode, estimate)
    I6 = inertia_from_pi(piHat)

    gBody = H[0:3, 0:3].T @ gravity
    Wg = np.concatenate([skew(piHat[1:4]) @ gBody, piHat[0] * gBody])

    s = V - Vr
    LinvS = np.linalg.solve(Lambda, s)
    sNorm_sq = max(0.0, float(s @ LinvS))
    sNorm = np.sqrt(sNorm_sq)

    alpha = float(cfg['alpha'])
    kd = float(cfg['kd'])
    ks = float(cfg['ks'])
    if sNorm == 0.0:
        D = np.zeros(6, dtype=float)
    else:
        D = (kd + ks * (sNorm ** (alpha - 1.0))) * LinvS

    # Commanded body wrench
    Y = regressor(H, V, Vr, VrDot, gravity, coriolis_form)
    W = Y @ piHat - D

    Vs = float(0.5 * (s @ (I6 @ s)))
    diagnostics = ControllerDiagnostics(
        He=He,
        Ve=Ve,
        eH=eH,
        eHdot=eHdot,
        Psi=psi,
        Vr=Vr,
        VrDot=VrDot,
        s=s,
        D=D,
        Wg=Wg,
        Y=Y,
        Vs=Vs,
        sNorm=sNorm,
    )

    estimate_next = estimate
    if dt_adapt is not None and dt_adapt > 0.0:
        if mode == "euclidean":
            gammaE = np.asarray(cfg['gammaE'], dtype=float).ravel()
            estimate_next = euclidean_step(piHat, Y.T @ s, np.diag(gammaE), dt_adapt)
        elif mode == "bregman":
            gammaB = float(cfg['gammaB'])
            G = pseudo_gradient(Y.T @ s)
            estimate_next = bregman_step(Jhat, G, gammaB, dt_adapt)

    return W, diagnostics, estimate_next
