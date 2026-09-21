"""Unit tests for Python persistence utilities (Task 7)."""

import json
import pytest
import numpy as np
from pathlib import Path

from agc.sim.default_scenario import default_scenario
from agc.sim.run_scenario import run_scenario
from agc.sim.metrics import compute_metrics
from agc.io.persistence import (
    save_run,
    load_run,
    save_batch_suite,
    resolve_result_suite,
    default_results_root,
    save_best_gain,
    load_best_gain,
)


def test_default_result_roots_use_best_gain_inplace_and_timestamped_layouts():
    repo_root = "/repo"

    assert default_results_root(repo_root) == Path(repo_root) / "results" / "optimization" / "best-gain"
    assert default_results_root(repo_root, inplace_save=True) == Path(repo_root) / "results" / "optimization" / "best-gain"
    assert default_results_root(repo_root, inplace_save=False, timestamp="20260922_143000") == (
        Path(repo_root) / "results" / "optimization" / "timestamped" / "20260922_143000"
    )


def test_best_gain_roundtrips_in_one_optimization_folder(tmp_path):
    gains = {"KRdiag": np.array([1.0, 2.0, 3.0]), "kd": 4.0}
    best_file = tmp_path / "best-gain" / "nominal_c1.json"

    save_best_gain(best_file, "nominal", "c1", gains, 2755.24, "sliding_dissipation")
    loaded = load_best_gain(best_file)

    assert loaded["mode"] == "nominal"
    assert loaded["coriolis"] == "c1"
    assert np.allclose(loaded["gains"]["KRdiag"], gains["KRdiag"])
    assert loaded["cost"] == pytest.approx(2755.24)
    assert loaded["stage"] == "sliding_dissipation"


def test_save_and_load_run_with_metadata(tmp_path):
    """Verify save_run produces run.npz and metadata.json and load_run restores them."""
    scen = default_scenario(duration=0.05)
    run, failure = run_scenario(scen)
    assert failure is None

    metrics = compute_metrics(run)
    save_dir = tmp_path / "run_test"
    save_run(str(save_dir), run, metrics, scen)

    assert (save_dir / "run.npz").is_file()
    assert (save_dir / "metadata.json").is_file()

    # Verify metadata.json content
    with open(save_dir / "metadata.json", "r", encoding="utf-8") as f:
        meta = json.load(f)
    assert meta["schemaVersion"] == "1.0"
    assert meta["variant"] == "bregman_c1"
    assert meta["completionStatus"] == "completed"
    assert "metrics" in meta and meta["metrics"] is not None

    # Load run
    loaded = load_run(str(save_dir))
    assert np.allclose(loaded["H"], run["H"])
    assert np.allclose(loaded["wrench"], run["wrench"])
    assert loaded["metadata"]["variant"] == "bregman_c1"
    assert loaded["metrics"]["positionRMSE"] == metrics["positionRMSE"]


def test_save_run_with_failure(tmp_path):
    """Verify save_run records failure metadata when a run fails."""
    scen = default_scenario(duration=0.05)
    run, _ = run_scenario(scen)

    failure_info = {"identifier": "TestError", "message": "Simulated failure", "time": 0.02}
    save_dir = tmp_path / "failed_run"
    save_run(str(save_dir), run, None, scen, failure=failure_info)

    assert (save_dir / "metadata.json").is_file()
    with open(save_dir / "metadata.json", "r", encoding="utf-8") as f:
        meta = json.load(f)
    assert meta["completionStatus"] == "failed"
    assert meta["failure"]["identifier"] == "TestError"
    assert meta["metrics"] is None

    loaded = load_run(str(save_dir))
    assert loaded["failure"]["identifier"] == "TestError"


def test_save_batch_suite_and_resolve(tmp_path):
    """Verify save_batch_suite writes manifest.json and per-variant subdirs, resolved correctly."""
    variants = [("nominal", "c1"), ("bregman", "c2")]
    scenarios = [default_scenario(mode=m, coriolis=c, duration=0.02) for m, c in variants]

    runs = []
    metrics_list = []
    failures = []
    for sc in scenarios:
        r, f = run_scenario(sc)
        runs.append(r)
        failures.append(f)
        metrics_list.append(compute_metrics(r) if f is None else None)

    batch = {"runs": runs, "metrics": metrics_list, "failures": failures}
    suite_dir = tmp_path / "test_suite"
    summary = save_batch_suite(str(suite_dir), scenarios, batch)

    assert len(summary) == 2
    assert (suite_dir / "manifest.json").is_file()
    assert (suite_dir / "nominal_c1" / "run.npz").is_file()
    assert (suite_dir / "bregman_c2" / "run.npz").is_file()

    with open(suite_dir / "manifest.json", "r", encoding="utf-8") as f:
        manifest = json.load(f)
    assert manifest["schemaVersion"] == "1.0"
    assert manifest["successCount"] == 2

    # Test resolve_result_suite
    resolved = resolve_result_suite(str(suite_dir))
    assert resolved == suite_dir

    resolved_from_sub = resolve_result_suite(str(suite_dir / "nominal_c1"))
    assert resolved_from_sub == suite_dir
