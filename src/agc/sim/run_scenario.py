"""Deterministic multi-rate closed-loop simulation loop with PyBullet plant."""

from typing import Tuple, Optional, Dict, Any
import warnings
import numpy as np

from ..paper.controller import controller
from ..math.inertia import pi_from_pseudo, pseudo_from_pi, inertia_from_pi
from ..viz.live_telemetry import make_snapshot
from .validation import validate_scenario


def estimate_to_pi(mode: str, estimate: Any) -> np.ndarray:
    """Convert mode-specific estimator state to 10-parameter vector."""
    if mode.lower() == "bregman":
        return pi_from_pseudo(estimate)
    return np.asarray(estimate, dtype=float).ravel()


def controller_estimate(cfg: dict, estimate: Any, active_pi: np.ndarray) -> Any:
    """Select an explicitly configured known-inertia estimate for nominal validation."""
    if str(cfg.get("knownInertiaSchedule", "")).lower() == "active-plant":
        return np.asarray(active_pi, dtype=float).copy()
    return estimate


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
    validate_scenario(scenario)

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
        "VdotDesired": np.zeros((n, 6), dtype=float),
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
    ground_z = float(scenario.get("groundZ", 0.0))
    ground_style = str(scenario.get("groundStyle", "arena"))
    gates_mode = str(scenario.get("gatesMode", "lemniscate"))
    cam_mode = str(scenario.get("camMode", "chase"))
    enable_osd = bool(scenario.get("enableOsd", False))
    drone_type = str(scenario.get("droneType", "pybullet_drones"))

    from ..plant.pybullet_plant import PyBulletPlant

    plant = PyBulletPlant(
        pi=scenario["plantPi"],
        gravity=scenario["plantGravity"],
        dt=dt_plant,
        gui=gui,
        payload=payload_info,
        release_time=release_time,
        sim_speed=sim_speed,
        enable_pacing=enable_pacing,
        ground_z=ground_z,
        ground_style=ground_style,
        gates_mode=gates_mode,
        cam_mode=cam_mode,
        enable_osd=enable_osd,
        drone_type=drone_type,
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
    last_payload_dropped = False
    viz_every = max(1, int(round((1.0 / 30.0) / dt_plant)))

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
                # Abort runaway candidates early during gain optimization. A
                # completed trajectory with enormous state values is not useful
                # evidence and can make each PSO worker spend minutes on one
                # unstable candidate.
                if (
                    np.linalg.norm(state["V"]) > 1.0e3
                    or np.max(np.abs(state["H"])) > 1.0e6
                ):
                    raise FloatingPointError("Trajectory diverged: state magnitude exceeded safety bound.")

                # Determine active plant inertial parameters
                if payload_drop is None:
                    active_pi = scenario["plantPi"]
                elif t < payload_drop["releaseTime"]:
                    active_pi = payload_drop["loadedPi"]
                else:
                    active_pi = payload_drop["barePi"]

                # Instantaneous diagnostics, independent of the zero-order-held command.
                estimate_for_control = controller_estimate(scenario["controller"], estimate, active_pi)
                evaluated_wrench, diagnostics, _ = controller(
                    state, desired, scenario["controller"], estimate_for_control, dt_adapt=None
                )
                if k % control_every == 0:
                    last_wrench = evaluated_wrench
                last_s = diagnostics.s
                last_psi = diagnostics.Psi
                last_vs = float(.5 * last_s @ inertia_from_pi(active_pi) @ last_s)

                # Log pre-propagation state and signals
                run["H"][k] = state["H"]
                run["V"][k] = state["V"]
                run["Hdesired"][k] = desired["H"]
                run["Vdesired"][k] = desired["V"]
                run["VdotDesired"][k] = desired.get("Vdot", np.zeros(6))
                run["wrench"][k] = last_wrench
                run["s"][k] = last_s
                run["Psi"][k] = last_psi
                run["Vs"][k] = last_vs
                run["activePlantPi"][k] = active_pi
                run["estimatePi"][k] = estimate_to_pi(mode, estimate_for_control)

                J_sym = estimate if mode == "bregman" else pseudo_from_pi(run["estimatePi"][k])
                run["minPseudoEigenvalue"][k] = float(np.linalg.eigvalsh(J_sym).min())

                if k % adapt_every == 0 and mode != "nominal" and k < n - 1:
                    _, _, estimate = controller(
                        state, desired, scenario["controller"], estimate, dt_adapt=dt_adapt
                    )

                logged_steps = k + 1

                # Update live visualization at a bounded rate, independently of physics.
                if gui and (k % viz_every == 0 or k == n - 1):
                    event = None
                    if plant.payload_dropped and not last_payload_dropped:
                        event = "payload_drop"
                    snapshot = make_snapshot(
                        t=t,
                        actual_H=state["H"],
                        desired_H=desired["H"],
                        actual_V=state["V"],
                        desired_V=desired["V"],
                        sliding=last_s,
                        wrench=last_wrench,
                        estimate_pi=run["estimatePi"][k],
                        true_pi=active_pi,
                        payload_dropped=plant.payload_dropped,
                        min_pseudo_eigenvalue=run["minPseudoEigenvalue"][k],
                        event=event,
                    )
                    plant.update_viz(snapshot=snapshot, step_idx=k, mode=mode, coriolis=coriolis)
                last_payload_dropped = plant.payload_dropped

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
        for key in ["t", "H", "V", "Hdesired", "Vdesired", "VdotDesired", "wrench", "s", "activePlantPi", "Psi", "Vs", "estimatePi", "minPseudoEigenvalue"]:
            run[key] = run[key][:logged_steps]

    finally:
        plant.close()

    run["finalEstimate"] = estimate
    return run, failure
