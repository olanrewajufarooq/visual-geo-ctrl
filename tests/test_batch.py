"""Unit tests for batch execution and failure isolation (Task 8)."""

import pytest
from agc.sim.default_scenario import default_scenario
from agc.batch.run_batch import run_batch


def test_batch_serial_execution_and_ordering():
    """Verify batch runs preserve exact input scenario ordering."""
    variants = [("nominal", "c1"), ("bregman", "c1")]
    scenarios = [default_scenario(mode=m, coriolis=c, duration=0.04) for m, c in variants]

    res = run_batch(scenarios, parallel=False, verbose=False)
    assert res["successCount"] == 2
    assert len(res["runs"]) == 2
    assert res["runs"][0]["mode"] == "nominal"
    assert res["runs"][1]["mode"] == "bregman"


def test_batch_parallel_execution():
    """Verify parallel batch execution runs across worker processes."""
    variants = [("nominal", "c1"), ("euclidean", "c2")]
    scenarios = [default_scenario(mode=m, coriolis=c, duration=0.04) for m, c in variants]

    res = run_batch(scenarios, parallel=True, max_workers=2, verbose=False)
    assert res["successCount"] == 2
    assert len(res["runs"]) == 2
    assert res["parallel"] is True


def test_batch_failure_isolation():
    """Verify that a failing scenario does not crash or abort other batch scenarios."""
    scen_good = default_scenario(mode="nominal", coriolis="c1", duration=0.04)
    scen_bad = default_scenario(mode="nominal", coriolis="c1", duration=0.04)
    # Sabotage scen_bad by injecting NaN into plantPi
    scen_bad["plantPi"][0] = float("nan")

    res = run_batch([scen_good, scen_bad], parallel=False, verbose=False)
    assert res["successCount"] == 1
    assert res["failures"][0] is None
    assert res["failures"][1] is not None
    assert res["metrics"][0] is not None
    assert res["metrics"][1] is None
