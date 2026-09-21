"""End-to-end integration and workflow tests."""

import os
import sys
from pathlib import Path
import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from agc.sim.default_scenario import default_scenario
from agc.sim.run_scenario import run_scenario
from agc.sim.metrics import compute_metrics
from agc.io.persistence import save_run, load_run
from agc.viz.paper_figures import export_run_figures


def test_end_to_end_simulation_and_figure_export(tmp_path):
    # Run short 0.05s test for euclidean mode
    scenario = default_scenario(
        replay_id="lemniscate_01_auto",
        mode="euclidean",
        coriolis="c1",
        duration=0.05,
        gui=False,
    )

    run, failure = run_scenario(scenario)
    assert failure is None

    metrics = compute_metrics(run)
    assert metrics["positionRMSE"] < 0.5

    # Test saving and loading
    save_dir = str(tmp_path / "test_run")
    save_run(save_dir, run, metrics, scenario)
    loaded = load_run(save_dir)
    assert "t" in loaded
    assert "H" in loaded

    # Test figure export
    export_run_figures(run, save_dir)
    assert os.path.isfile(os.path.join(save_dir, "tracking_performance.png"))
    assert os.path.isfile(os.path.join(save_dir, "parameter_adaptation.png"))
