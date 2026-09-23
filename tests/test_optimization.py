"""Tests for AGC gain optimization module matching and extending MATLAB TestOptimization.m."""

import sys
from pathlib import Path
from unittest.mock import patch
import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from agc.opt.bounds import (
    gain_bounds,
    gain_block_indices,
    gain_optimization_stages,
    expand_scenario_selection,
)
from agc.opt.encoding import (
    encode_scenario_gains,
    apply_scenario_gains,
    apply_gain_block,
    round_gains,
)
from agc.opt.objective import (
    aggregate_scenario_records,
    objective_scales,
    objective_weights,
    optimization_options,
    best_feasible_candidate,
)
from agc.opt.bregman_profile import bregman_gamma_grid
from agc.opt.pso import ParticleSwarmOptimizer
from agc.opt.de import DifferentialEvolutionOptimizer
from agc.opt.staged_optimizer import promote_gains_to_registry
from agc.config.manual_gains import manual_gains
from agc.sim.default_scenario import default_scenario
from agc.opt.staged_optimizer import default_training_conditions


def test_default_convergence_uses_ten_stalled_iterations_at_one_milliunit():
    options = optimization_options()
    assert np.isclose(options["functionTolerance"], 1e-3)
    assert options["maxStallIterations"] == 10
    assert options["maxIterations"] == 20


def test_optimizer_defaults_use_shared_velocity_and_effort_weights():
    weights = objective_weights()
    options = optimization_options()

    expected = {
        "position": 2.0,
        "attitude": 2.0,
        "mass": 2.5,
        "cog": 2.5,
        "linVel": 0.5,
        "angVel": 0.5,
        "inertia": 1.5,
        "effort": 0.5,
        "failure": 1e6,
    }
    for k, v in expected.items():
        assert np.isclose(weights[k], v)
        assert np.isclose(options["weights"][k], v)


def test_optimizer_uses_explicit_physical_error_scales():
    scales = objective_scales()
    expected = {
        "position": 0.05,
        "attitude": 0.05,
        "mass": 0.05,
        "cog": 0.01,
        "linVel": 0.20,
        "angVel": 0.50,
        "inertia": 0.05,
        "effort": 50.0,
    }
    for k, v in expected.items():
        assert np.isclose(scales[k], v)


def test_supplied_convergence_settings_override_defaults():
    options = optimization_options({"functionTolerance": 2e-4, "maxStallIterations": 7})
    assert np.isclose(options["functionTolerance"], 2e-4)
    assert options["maxStallIterations"] == 7


def test_blank_selectors_expand_to_complete_mode_and_factorization_matrix():
    variants = expand_scenario_selection("", "")
    expected = [
        ("nominal", "c1"),
        ("nominal", "c2"),
        ("euclidean", "c1"),
        ("euclidean", "c2"),
        ("bregman", "c1"),
        ("bregman", "c2"),
    ]
    assert variants == expected


def test_list_selectors_form_their_cartesian_product():
    variants = expand_scenario_selection(["bregman", "euclidean"], "c2")
    assert variants == [("bregman", "c2"), ("euclidean", "c2")]


def test_gain_blocks_cover_and_partition_adaptive_controllers():
    all_indices = gain_block_indices("euclidean", "all")
    tracking = gain_block_indices("euclidean", "tracking")
    sliding_diss = gain_block_indices("euclidean", "sliding_dissipation")
    sliding_metric = gain_block_indices("euclidean", "sliding_metric")
    dissipation = gain_block_indices("euclidean", "dissipation")
    adaptive = gain_block_indices("euclidean", "adaptive")

    assert all_indices == list(range(25))
    assert tracking == list(range(0, 6))
    assert sliding_diss == list(range(6, 15))
    assert sliding_metric == list(range(6, 12))
    assert dissipation == list(range(12, 15))
    assert adaptive == list(range(15, 25))

    # Test disjoint partitions
    assert set(tracking).isdisjoint(set(sliding_metric))
    assert set(sliding_metric).isdisjoint(set(dissipation))
    assert set(dissipation).isdisjoint(set(adaptive))

    # Test complete union
    assert sorted(tracking + sliding_metric + dissipation + adaptive) == all_indices


def test_bregman_grid_includes_bounds_and_seed_values():
    grid = bregman_gamma_grid([1e-3, 3e-4, 1e-5])
    assert np.isclose(grid[0], -5.0, atol=1e-12)
    assert np.isclose(grid[-1], -1.0, atol=1e-12)
    assert np.any(np.isclose(grid, np.log10(3e-4), atol=1e-12))
    assert len(grid) == len(np.unique(grid))


def test_incumbent_selection_retains_feasible_lower_cost_candidate():
    incumbent = {"candidate": [1, 2], "cost": 4.0, "failed": False, "label": "incumbent"}
    worse = {"candidate": [3, 4], "cost": 5.0, "failed": False, "label": "stage"}
    failed = {"candidate": [5, 6], "cost": 1.0, "failed": True, "label": "stage"}
    better = {"candidate": [7, 8], "cost": 3.0, "failed": False, "label": "stage"}

    assert best_feasible_candidate(incumbent, worse) == incumbent
    assert best_feasible_candidate(incumbent, failed) == incumbent
    assert best_feasible_candidate(incumbent, better) == better


def test_training_conditions_default_to_three_replays_and_two_payloads():
    conditions = default_training_conditions()

    assert [(c["replayId"], c["payloadProfile"]) for c in conditions] == [
        ("lemniscate_02_auto", "flat_light"),
        ("lemniscate_02_auto", "tall_heavy"),
        ("lemniscate_03_auto", "flat_light"),
        ("lemniscate_03_auto", "tall_heavy"),
        ("lemniscate_04_auto", "flat_light"),
        ("lemniscate_04_auto", "tall_heavy"),
    ]


def test_training_conditions_reject_explicitly_empty_overrides():
    with pytest.raises(ValueError, match="at least one replay"):
        default_training_conditions(replay_ids=[])
    with pytest.raises(ValueError, match="at least one replay"):
        default_training_conditions(payload_profiles=[])


def test_training_cost_averages_all_finite_conditions_and_rejects_any_failure():
    records = [
        {"cost": 2.0, "failed": False, "label": "candidate"},
        {"cost": 8.0, "failed": False, "label": "candidate"},
    ]
    aggregate = aggregate_scenario_records(records, label="candidate")

    assert aggregate["cost"] == pytest.approx(5.0)
    assert aggregate["failed"] is False
    assert aggregate["conditionRecords"] == records

    failed = aggregate_scenario_records(records + [{"cost": 1e6, "failed": True}], label="candidate")
    assert failed["failed"] is True
    assert failed["cost"] == pytest.approx(1e6)


def test_staged_schedule_hierarchical_and_classic():
    assert gain_optimization_stages("nominal", "classic") == ["all"]
    assert gain_optimization_stages("euclidean", "classic") == ["all", "nonadaptive", "adaptive", "all"]
    assert gain_optimization_stages("bregman", "classic") == ["all", "nonadaptive", "adaptive", "all"]

    assert gain_optimization_stages("nominal", "hierarchical") == [
        "all",
        "tracking",
        "sliding_dissipation",
        "sliding_metric",
        "dissipation",
        "all",
    ]
    assert gain_optimization_stages("bregman", "hierarchical") == [
        "all",
        "tracking",
        "sliding_dissipation",
        "sliding_metric",
        "dissipation",
        "adaptive",
        "all",
    ]


def test_encode_decode_roundtrip():
    scenario = default_scenario(mode="bregman", coriolis="c1", duration=2.0)
    cand = encode_scenario_gains(scenario)
    assert len(cand) == 16

    new_sc = apply_scenario_gains(cand, scenario)
    cand2 = encode_scenario_gains(new_sc)
    assert np.allclose(cand, cand2, atol=1e-12)


def test_promote_gains_loads_python_registry_without_relative_import_error():
    gains = manual_gains("bregman", "c1")

    with patch.object(Path, "write_text") as write_text:
        promote_gains_to_registry(
            "bregman",
            "c1",
            gains,
            target_file="optimized_gains_test.py",
        )

    write_text.assert_called_once()


def test_pso_sphere_function_convergence():
    np.random.seed(42)

    def sphere(x):
        return float(np.sum(x**2))

    lb = np.array([-5.0, -5.0, -5.0])
    ub = np.array([5.0, 5.0, 5.0])
    pso = ParticleSwarmOptimizer(
        cost_func=sphere,
        lower_bound=lb,
        upper_bound=ub,
        swarm_size=30,
        max_iter=50,
        initial_points=np.array([[1.0, 1.0, 1.0]]),
        parallel=False,
        verbose=False,
    )
    best_x, best_cost, hist = pso.optimize()
    assert best_cost < 0.1
    assert best_cost < hist[0]  # strictly improved from initial
    assert np.allclose(best_x, np.zeros(3), atol=0.25)


def test_de_explicitly_uses_deferred_updates_with_workers():
    captured = {}

    class Result:
        x = np.array([0.0])
        fun = 0.0

    def fake_differential_evolution(*args, **kwargs):
        captured.update(kwargs)
        return Result()

    with patch("agc.opt.de.differential_evolution", fake_differential_evolution):
        optimizer = DifferentialEvolutionOptimizer(
            cost_func=lambda x: float(np.sum(x**2)),
            lower_bound=np.array([-1.0]),
            upper_bound=np.array([1.0]),
            parallel=True,
            max_workers=2,
            verbose=False,
        )
        optimizer.optimize()

    assert captured["updating"] == "deferred"


def test_de_progress_notes_show_iteration_number_without_extra_evaluations():
    """DE callback reads intermediate_result.fun — no extra cost_func call per iteration."""
    import io
    from contextlib import redirect_stdout

    call_count = [0]

    def sphere(x):
        call_count[0] += 1
        return float(np.sum(x**2))

    opt = DifferentialEvolutionOptimizer(
        cost_func=sphere,
        lower_bound=np.array([-5.0, -5.0]),
        upper_bound=np.array([5.0, 5.0]),
        pop_size=5,
        max_iter=3,
        parallel=False,
        verbose=True,
        seed=0,
    )

    buf = io.StringIO()
    with redirect_stdout(buf):
        best_x, best_cost, hist = opt.optimize()

    output = buf.getvalue()
    iter_lines = [line for line in output.splitlines() if "DE Iter" in line]

    # Iteration counter must appear in at least one printed line
    assert len(iter_lines) >= 1, "No 'DE Iter' lines were printed"
    assert any("1/" in line for line in iter_lines), "Iteration number missing from output"

    # History is populated by the callback (at most max_iter entries)
    assert 1 <= len(hist) <= 3

    # Guard against regression: total cost_func calls must not exceed what
    # SciPy's population evaluations require — no extra call per callback.
    # pop_size=5, dim=2, max_iter=3 → generous upper bound of 5*2*(3+2)=50
    assert call_count[0] <= 50, (
        f"cost_func called {call_count[0]} times — callback may be re-evaluating"
    )
