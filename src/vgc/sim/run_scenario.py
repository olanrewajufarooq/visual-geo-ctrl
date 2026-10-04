"""Deterministic nominal closed-loop simulation with the PyBullet plant."""

from typing import Optional, Tuple
import warnings

import numpy as np

from ..paper.controller import controller
from .validation import validate_scenario


def run_scenario(scenario: dict) -> Tuple[dict, Optional[dict]]:
    """Execute one nominal multi-rate simulation and return its run log."""
    validate_scenario(scenario)
    duration = float(scenario["duration"])
    dt_plant = float(scenario["dtPlant"])
    dt_control = float(scenario["dtControl"])
    n_steps = int(round(duration / dt_plant))
    t_arr = np.arange(n_steps + 1) * dt_plant
    n = len(t_arr)
    run = {
        "t": t_arr,
        "H": np.zeros((n, 4, 4)), "V": np.zeros((n, 6)),
        "Hdesired": np.zeros((n, 4, 4)), "Vdesired": np.zeros((n, 6)),
        "VdotDesired": np.zeros((n, 6)), "wrench": np.zeros((n, 6)),
        "s": np.zeros((n, 6)), "Psi": np.zeros(n), "Vs": np.zeros(n),
        "mode": "nominal", "coriolis": str(scenario["controller"]["coriolis"]).lower(),
    }

    from ..plant.pybullet_plant import PyBulletPlant
    plant = PyBulletPlant(
        pi=scenario["plantPi"], gravity=scenario["plantGravity"], dt=dt_plant,
        gui=bool(scenario.get("gui", False)), sim_speed=float(scenario.get("simSpeed", 1.0)),
        enable_pacing=bool(scenario.get("enablePacing", True)), ground_z=float(scenario.get("groundZ", 0.0)),
        ground_style=str(scenario.get("groundStyle", "arena")), gates_mode=str(scenario.get("gatesMode", "lemniscate")),
        cam_mode=str(scenario.get("camMode", "chase")), enable_osd=bool(scenario.get("enableOsd", False)),
        drone_type=str(scenario.get("droneType", "pybullet_drones")),
    )
    state = scenario["initial"]
    plant.set_state(state["H"], state["V"])
    control_every = int(round(dt_control / dt_plant))
    last_wrench = np.zeros(6)
    last_diag = None
    viz_every = max(1, int(round((1.0 / 30.0) / dt_plant)))
    logged_steps = 0
    failure = None
    try:
        if scenario.get("gui"):
            plant.draw_reference_path(scenario["trajectory"], duration=duration)
        with warnings.catch_warnings(), np.errstate(all="ignore"):
            warnings.simplefilter("ignore", category=RuntimeWarning)
            for k, t in enumerate(t_arr):
                desired = scenario["trajectory"](t)
                if not np.all(np.isfinite(state["V"])) or not np.all(np.isfinite(state["H"])):
                    raise FloatingPointError("Trajectory diverged: non-finite plant state.")
                if k % control_every == 0:
                    last_wrench, last_diag = controller(state, desired, scenario["controller"])
                run["H"][k] = state["H"]
                run["V"][k] = state["V"]
                run["Hdesired"][k] = desired["H"]
                run["Vdesired"][k] = desired["V"]
                run["VdotDesired"][k] = desired.get("Vdot", np.zeros(6))
                run["wrench"][k] = last_wrench
                run["s"][k] = last_diag.s
                run["Psi"][k] = last_diag.Psi
                run["Vs"][k] = last_diag.Vs
                logged_steps = k + 1
                if scenario.get("gui") and (k % viz_every == 0 or k == n - 1):
                    from ..viz.live_telemetry import make_snapshot
                    snapshot = make_snapshot(
                        t=t, actual_H=state["H"], desired_H=desired["H"], actual_V=state["V"],
                        desired_V=desired["V"], sliding=last_diag.s, wrench=last_wrench,
                        estimate_pi=scenario["plantPi"], true_pi=scenario["plantPi"], event=None,
                    )
                    plant.update_viz(snapshot=snapshot, step_idx=k, mode="nominal", coriolis=run["coriolis"])
                if k < n - 1:
                    plant.apply_wrench(last_wrench)
                    plant.step(time=t)
                    state = plant.get_state()
    except Exception as exc:
        failure = {"identifier": type(exc).__name__, "message": str(exc), "time": float(t_arr[logged_steps])}
        for key in ("t", "H", "V", "Hdesired", "Vdesired", "VdotDesired", "wrench", "s", "Psi", "Vs"):
            run[key] = run[key][:logged_steps]
    finally:
        plant.close()
    return run, failure
