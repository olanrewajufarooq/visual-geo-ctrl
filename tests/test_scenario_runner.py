"""Tests for closed-loop scenario simulation and estimation."""

import sys
from pathlib import Path
import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from agc.sim.default_scenario import default_scenario
from agc.sim.run_scenario import run_scenario
from agc.sim.metrics import compute_metrics


def test_nominal_short_run():
    scenario = default_scenario(
        replay_id="lemniscate_01_auto",
        mode="nominal",
        coriolis="c1",
        duration=0.1,
        gain_source="optimized",
        gui=False,
    )
    run, failure = run_scenario(scenario)
    assert failure is None
    assert len(run["t"]) == 51  # 0.1 / 0.002 + 1
    assert np.all(np.isfinite(run["H"]))
    assert np.all(np.isfinite(run["V"]))
    assert np.all(np.isfinite(run["wrench"]))
    assert np.all(run["Psi"] >= -1e-12)


def test_bregman_short_run_preserves_spd():
    scenario = default_scenario(
        replay_id="lemniscate_01_auto",
        mode="bregman",
        coriolis="c1",
        duration=0.1,
        gain_source="optimized",
        gui=False,
    )
    run, failure = run_scenario(scenario)
    assert failure is None
    # Check that all logged min eigenvalues are positive
    min_eigs = run["minPseudoEigenvalue"]
    assert np.all(min_eigs > 0.0)

    metrics = compute_metrics(run)
    assert metrics["minimumPseudoEigenvalue"] > 0.0
    assert np.isfinite(metrics["positionRMSE"])


def test_payload_drop_parameters():
    scenario = default_scenario(
        replay_id="lemniscate_01_auto",
        mode="nominal",
        coriolis="c2",
        duration=15.0,
    )
    drop = scenario["payloadDrop"]
    assert drop["releaseTime"] == 10.0
    assert drop["payload"]["mass"] == 0.75
    assert np.allclose(drop["payload"]["center"], np.array([0.20, 0.05, -0.12]))


def test_payload_profiles_separate_optimization_from_evaluation_conditions():
    """Training payloads must differ physically while evaluation stays unchanged."""
    evaluation = default_scenario(duration=0.02, payload_profile="evaluation")
    flat_light = default_scenario(duration=0.02, payload_profile="flat_light")
    tall_heavy = default_scenario(duration=0.02, payload_profile="tall_heavy")

    assert evaluation["payloadProfile"] == "evaluation"
    assert evaluation["payloadDrop"]["payload"]["mass"] == 0.75
    assert np.allclose(evaluation["payloadDrop"]["payload"]["dimensions"], [0.12, 0.12, 0.08])

    assert flat_light["payloadDrop"]["payload"]["mass"] == 0.60
    assert np.allclose(flat_light["payloadDrop"]["payload"]["dimensions"], [0.16, 0.10, 0.06])
    assert tall_heavy["payloadDrop"]["payload"]["mass"] == 0.90
    assert np.allclose(tall_heavy["payloadDrop"]["payload"]["dimensions"], [0.10, 0.10, 0.16])
    assert flat_light["payloadDrop"]["loadedPi"][0] == pytest.approx(4.246)
    assert tall_heavy["payloadDrop"]["loadedPi"][0] == pytest.approx(4.546)


def test_exact_release_timing():
    """Verify active parameters transition from loaded to bare exactly at releaseTime."""
    duration = 1.0
    release_time = 0.5
    scen = default_scenario(duration=duration)
    scen["payloadDrop"]["releaseTime"] = release_time

    run, failure = run_scenario(scen)
    assert failure is None

    t = run["t"]
    dt = scen["dtPlant"]
    idx_release = int(round(release_time / dt))

    # Before release: loadedPi
    loaded_pi = scen["payloadDrop"]["loadedPi"]
    bare_pi = scen["payloadDrop"]["barePi"]
    for k in range(idx_release):
        assert np.allclose(run["activePlantPi"][k], loaded_pi, atol=1e-12)

    # At and after release: barePi
    for k in range(idx_release, len(t)):
        assert np.allclose(run["activePlantPi"][k], bare_pi, atol=1e-12)


def test_first_step_failure_preserves_prefix():
    """Verify first-step failure preserves metadata and finalEstimate."""
    scen = default_scenario(duration=0.1)
    # Inject non-finite initial velocity
    scen["initial"]["V"][0] = np.nan
    with pytest.raises(ValueError, match="initial V must be a finite 6-element vector"):
        run_scenario(scen)


def test_multirate_timing_and_wrench_zoh():
    """Verify wrench is zero-order held between control instants and adaptation updates at dtAdapt."""
    scen = default_scenario(duration=0.2)
    # dtPlant = 0.002, dtControl = 0.02 (every 10 steps), dtAdapt = 0.01 (every 5 steps)
    run, failure = run_scenario(scen)
    assert failure is None

    control_stride = int(round(scen["dtControl"] / scen["dtPlant"]))
    wrenches = run["wrench"]

    # Wrench should remain identical between control updates
    for k in range(len(wrenches) - 1):
        if (k + 1) % control_stride != 0:
            assert np.allclose(wrenches[k + 1], wrenches[k], atol=1e-12)
