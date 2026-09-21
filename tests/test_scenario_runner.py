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
