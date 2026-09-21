"""Deterministic multi-rate closed-loop simulation loop with PyBullet plant."""

from typing import Tuple, Optional, Dict, Any
import warnings
import numpy as np

from ..paper.controller import controller
from ..math.inertia import pi_from_pseudo
from ..plant.pybullet_plant import PyBulletPlant


def estimate_to_pi(mode: str, estimate: Any) -> np.ndarray:
    """Convert mode-specific estimator state to 10-parameter vector."""
    if mode.lower() == "bregman":
        return pi_from_pseudo(estimate)
    return np.asarray(estimate, dtype=float).ravel()


def run_scenario(scenario: dict) -> Tuple[dict, Optional[dict]]:
    """Execute one deterministic, multi-rate closed-loop simulation.

    Controller, estimator, and PyBullet plant use explicit fixed rates.
    The controller wrench is zero-order-held between controller sample instants.

    Returns:
    --------
    run : dict
        Simulation trajectory and diagnostic signals.
    failure : dict or None
        Failure metadata if an exception occurred, or None if successful.
    """
    duration = float(scenario["duration"])
    dt_plant = float(scenario["dtPlant"])
    dt_control = float(scenario["dtControl"])
    dt_adapt = float(scenario["dtAdaptation"])

    n_steps = int(round(duration / dt_plant))
    n = n_steps + 1
    t_arr = np.arange(n) * dt_plant

    mode = str(scenario["controller"]["mode"]).lower()
    coriolis = str(scenario["controller"]["coriolis"]).lower()

    # Preallocate replay log
    run = {
        "t": t_arr,
        "H": np.zeros((n, 4, 4), dtype=float),
        "V": np.zeros((n, 6), dtype=float),
        "Hdesired": np.zeros((n, 4, 4), dtype=float),
        "Vdesired": np.zeros((n, 6), dtype=float),
        "wrench": np.zeros((n, 6), dtype=float),
        "s": np.zeros((n, 6), dtype=float),
        "activePlantPi": np.zeros((n, 10), dtype=float),
        "Psi": np.zeros(n, dtype=float),
        "Vs": np.zeros(n, dtype=float),
        "estimatePi": np.zeros((n, 10), dtype=float),
        "minPseudoEigenvalue": np.full(n, np.nan, dtype=float),
        "mode": mode,
        "coriolis": coriolis,
    }

    # Instantiate PyBullet plant
    payload_drop = scenario.get("payloadDrop")
    payload_info = payload_drop["payload"] if payload_drop else None
    release_time = payload_drop["releaseTime"] if payload_drop else None
    gui = bool(scenario.get("gui", False))
    sim_speed = float(scenario.get("simSpeed", 1.0))
    enable_pacing = bool(scenario.get("enablePacing", True))

    plant = PyBulletPlant(
        pi=scenario["plantPi"],
        gravity=scenario["plantGravity"],
        dt=dt_plant,
        gui=gui,
        payload=payload_info,
        release_time=release_time,
        sim_speed=sim_speed,
        enable_pacing=enable_pacing,
    )

    if gui:
        plant.draw_reference_path(scenario["trajectory"], duration=duration)

    state = scenario["initial"]
    plant.set_state(state["H"], state["V"])
    estimate = scenario["initialEstimate"]

    control_every = int(round(dt_control / dt_plant))
    adapt_every = int(round(dt_adapt / dt_plant))

    last_wrench = np.zeros(6, dtype=float)
    last_s = np.zeros(6, dtype=float)
    last_psi = 0.0
    last_vs = 0.0

    logged_steps = 0
    failure_time = 0.0
    failure = None

    try:
        with warnings.catch_warnings(), np.errstate(all="ignore"):
            warnings.simplefilter("ignore", category=RuntimeWarning)
            for k in range(n):
                t = t_arr[k]
                failure_time = t
                desired = scenario["trajectory"](t)

                # Check for numerical divergence / NaN in plant state
                if not np.all(np.isfinite(state["V"])) or not np.all(np.isfinite(state["H"])):
                    raise FloatingPointError("Trajectory diverged: non-finite plant state.")

                # Determine active plant inertial parameters
                if payload_drop is None:
                    active_pi = scenario["plantPi"]
                elif t < payload_drop["releaseTime"]:
                    active_pi = payload_drop["loadedPi"]
                else:
                    active_pi = payload_drop["barePi"]

                # Evaluate and hold wrench only at controller sample instants
                if k % control_every == 0:
                    last_wrench, diagnostics, _ = controller(
                        state, desired, scenario["controller"], estimate, dt_adapt=None
                    )
                    last_s = diagnostics.s
                    last_psi = diagnostics.Psi
                    last_vs = diagnostics.Vs

                # Evaluate parameter update at adaptation sample instants
                if k % adapt_every == 0 and mode != "nominal":
                    _, _, estimate = controller(
                        state, desired, scenario["controller"], estimate, dt_adapt=dt_adapt
                    )

                # Log pre-propagation state and signals
                run["H"][k] = state["H"]
                run["V"][k] = state["V"]
                run["Hdesired"][k] = desired["H"]
                run["Vdesired"][k] = desired["V"]
                run["wrench"][k] = last_wrench
                run["s"][k] = last_s
                run["Psi"][k] = last_psi
                run["Vs"][k] = last_vs
                run["activePlantPi"][k] = active_pi
                run["estimatePi"][k] = estimate_to_pi(mode, estimate)

                if mode == "bregman":
                    J_sym = 0.5 * (estimate + estimate.T)
                    eigvals = np.linalg.eigvalsh(J_sym)
                    run["minPseudoEigenvalue"][k] = float(np.min(eigvals))

                logged_steps = k + 1

                # Update live GUI 3D visualizer and pacing if enabled
                if gui:
                    pos_err = float(np.linalg.norm(state["H"][0:3, 3] - desired["H"][0:3, 3]))
                    s_norm = float(np.linalg.norm(last_s))
                    est_m = float(run["estimatePi"][k, 0])
                    true_m = float(active_pi[0])
                    plant.update_viz(
                        t=t,
                        pos_err=pos_err,
                        s_norm=s_norm,
                        est_m=est_m,
                        true_m=true_m,
                        step_idx=k,
                    )

                # Advance physics plant
                if k < n - 1:
                    plant.apply_wrench(last_wrench)
                    plant.step(time=t)
                    state = plant.get_state()

    except Exception as exc:
        failure = {
            "identifier": type(exc).__name__,
            "message": str(exc),
            "time": failure_time,
        }
        # Trim logged run to finite steps
        for key in ["t", "H", "V", "Hdesired", "Vdesired", "wrench", "s", "activePlantPi", "Psi", "Vs", "estimatePi", "minPseudoEigenvalue"]:
            run[key] = run[key][:logged_steps]

    finally:
        plant.close()

    run["finalEstimate"] = estimate
    return run, failure
