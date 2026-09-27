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
    evaluate_scenario_candidate,
    evaluate_scenario_set_candidate,
    objective_scales,
    objective_weights,
    optimization_options,
    best_feasible_candidate,
)
from agc.opt.bregman_profile import bregman_gamma_grid
from agc.opt.pso import ParticleSwarmOptimizer
from agc.opt.staged_optimizer import promote_gains_to_registry
from agc.config.manual_gains import manual_gains
from agc.sim.default_scenario import default_scenario
from agc.opt.staged_optimizer import default_training_conditions
from agc.opt.staged_optimizer import _optimization_context


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
        "forceEffort": 0.25,
        "torqueEffort": 0.25,
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
        "force": 50.0,
        "torque": 5.0,
    }
    for k, v in expected.items():
        assert np.isclose(scales[k], v)


def test_supplied_convergence_settings_override_defaults():
    options = optimization_options({"functionTolerance": 2e-4, "maxStallIterations": 7})
    assert np.isclose(options["functionTolerance"], 2e-4)
    assert options["maxStallIterations"] == 7


def test_blank_selectors_expand_to_unique_optimization_modes():
    modes = expand_scenario_selection("", "")
    assert modes == ["nominal", "adaptive"]
    assert expand_scenario_selection(None, None) == ["nominal", "adaptive"]
    assert expand_scenario_selection("all") == ["nominal", "adaptive"]


def test_mode_selectors_validate_and_filter():
    assert expand_scenario_selection("nominal") == ["nominal"]
    assert expand_scenario_selection(["adaptive"]) == ["adaptive"]
    with pytest.raises(ValueError, match="Unsupported mode selector"):
        expand_scenario_selection("invalid_mode")


def test_gain_blocks_cover_and_partition_adaptive_controllers():
    all_indices = gain_block_indices("adaptive", "all")
    tracking = gain_block_indices("adaptive", "tracking")
    sliding_diss = gain_block_indices("adaptive", "sliding_dissipation")
    sliding_metric = gain_block_indices("adaptive", "sliding_metric")
    dissipation = gain_block_indices("adaptive", "dissipation")
    adaptive = gain_block_indices("adaptive", "adaptive")

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

    # Test nominal partitions (15 parameters)
    nom_all = gain_block_indices("nominal", "all")
    assert nom_all == list(range(15))


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


def test_training_conditions_default_to_two_replays_two_payloads_two_coriolis():
    conditions = default_training_conditions()

    assert len(conditions) == 8
    assert [(c["replayId"], c["payloadProfile"], c["coriolis"]) for c in conditions] == [
        ("lemniscate_02_auto", "flat_light", "lc"),
        ("lemniscate_02_auto", "flat_light", "rb"),
        ("lemniscate_02_auto", "tall_heavy", "lc"),
        ("lemniscate_02_auto", "tall_heavy", "rb"),
        ("lemniscate_03_auto", "flat_light", "lc"),
        ("lemniscate_03_auto", "flat_light", "rb"),
        ("lemniscate_03_auto", "tall_heavy", "lc"),
        ("lemniscate_03_auto", "tall_heavy", "rb"),
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


def test_scenario_set_objective_scores_every_condition_and_uses_arithmetic_mean(monkeypatch):
    from agc.opt import objective

    calls = []
    costs = iter([1.0, 5.0, 9.0])

    def score_one(candidate, scenario, weights=None, label="candidate"):
        calls.append(scenario["id"])
        return {"cost": next(costs), "failed": False, "candidate": [1.0], "gains": {}}

    monkeypatch.setattr(objective, "evaluate_scenario_candidate", score_one)
    scenarios = [{"id": "a"}, {"id": "b"}, {"id": "c"}]
    aggregate = evaluate_scenario_set_candidate(np.array([1.0]), scenarios)

    assert calls == ["a", "b", "c"]
    assert aggregate["cost"] == pytest.approx((1.0 + 5.0 + 9.0) / 3.0)
    assert len(aggregate["conditionRecords"]) == len(scenarios)


def test_objective_uses_separately_scaled_force_and_torque_effort(monkeypatch):
    from agc.opt import objective

    scenario = default_scenario(mode="nominal", duration=1.0)
    candidate = encode_scenario_gains(scenario)
    H = np.repeat(np.eye(4)[None], 2, axis=0)
    pi = np.array([2.0, 0., 0., 0., .2, .2, .2, 0., 0., 0.])
    run = {
        "t": np.array([0., 1.]), "H": H, "Hdesired": H.copy(),
        "V": np.zeros((2, 6)), "Vdesired": np.zeros((2, 6)),
        "wrench": np.tile([3., 4., 0., 0., 0., 50.], (2, 1)),
        "s": np.zeros((2, 6)), "Psi": np.zeros(2), "Vs": np.zeros(2),
        "minPseudoEigenvalue": np.array([1., 1.]),
        "activePlantPi": np.tile(pi, (2, 1)), "estimatePi": np.tile(pi, (2, 1)),
    }
    monkeypatch.setattr(objective, "run_scenario", lambda scenario: (run, None))
    result = evaluate_scenario_candidate(candidate, scenario)
    assert result["metrics"]["forceRMS"] == pytest.approx(50.0)
    assert result["metrics"]["torqueRMS"] == pytest.approx(5.0)
    assert result["cost"] == pytest.approx(0.5)


def test_staged_schedule_hierarchical_and_classic():
    assert gain_optimization_stages("nominal", "classic") == ["all"]
    assert gain_optimization_stages("adaptive", "classic") == ["all", "nonadaptive", "adaptive"]

    assert gain_optimization_stages("nominal", "hierarchical") == [
        "all",
        "tracking",
        "sliding_dissipation",
        "sliding_metric",
        "dissipation",
        "all",
    ]
    # Adaptive hierarchical must end in 'adaptive' (omitting the final 'all' to avoid estimator bias)
    assert gain_optimization_stages("adaptive", "hierarchical") == [
        "all",
        "tracking",
        "sliding_dissipation",
        "sliding_metric",
        "dissipation",
        "adaptive",
    ]


def test_encode_decode_roundtrip():
    scenario = default_scenario(mode="bregman", coriolis="lc", duration=2.0)
    cand = encode_scenario_gains(scenario)
    assert len(cand) == 16

    new_sc = apply_scenario_gains(cand, scenario)
    cand2 = encode_scenario_gains(new_sc)
    assert np.allclose(cand, cand2, atol=1e-12)


def test_applying_gains_updates_default_reciprocal_lambda_s_but_keeps_explicit_metric():
    scenario = default_scenario(mode="nominal", duration=2.0)
    candidate = encode_scenario_gains(scenario)
    candidate[6] += 0.2
    updated = apply_scenario_gains(candidate, scenario)
    np.testing.assert_allclose(updated["controller"]["Lambda_s"], np.linalg.inv(updated["controller"]["Lambda"]))

    scenario["controller"]["Lambda_s"] = np.diag([2., 3., 4., 5., 6., 7.])
    explicit = apply_scenario_gains(candidate, scenario)
    np.testing.assert_array_equal(explicit["controller"]["Lambda_s"], scenario["controller"]["Lambda_s"])


def test_promote_gains_loads_python_registry_without_relative_import_error():
    gains = manual_gains("bregman", "lc")
    import agc.opt.staged_optimizer as staged_optimizer

    with patch.object(staged_optimizer, "atomic_write_text") as write_text:
        promote_gains_to_registry(
            "bregman",
            "lc",
            gains,
            target_file="optimized_gains_test.py",
        )

    write_text.assert_called_once()


def test_promoting_changed_tracking_gains_invalidates_adaptive_costs(monkeypatch):
    gains = manual_gains("nominal", "lc")
    import agc.opt.staged_optimizer as staged_optimizer
    captured = {}
    monkeypatch.setattr(staged_optimizer, "atomic_write_text", lambda path, content: captured.update(content=content))
    promote_gains_to_registry(
        "nominal", "all", gains,
        metadata={"cost": 1.0, "stage": "all", "contextHash": "new-context"},
        target_file="unused_optimized_gains.py",
    )
    generated = captured["content"]
    assert '"optimizationCost": None' in generated
    assert '"optimizationMetadata": None' in generated
    assert "'contextHash': 'new-context'" in generated
    namespace = {}
    exec(compile(generated, "optimized_gains.py", "exec"), namespace)
    assert namespace["optimized_gains"]("nominal")["optimizationMetadata"]["contextHash"] == "new-context"
    assert namespace["optimized_gains"]("euclidean")["optimizationCost"] is None


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


def test_pso_stall_early_stopping():
    opt = ParticleSwarmOptimizer(
        cost_func=lambda x: 1.0,
        lower_bound=np.array([-1.0]),
        upper_bound=np.array([1.0]),
        swarm_size=5,
        max_iter=50,
        max_stall=3,
        tol=1e-3,
        parallel=False,
        verbose=False,
        seed=0,
    )
    best_x, best_cost, hist = opt.optimize()
    assert len(hist) <= 5


def test_pso_resumes_checkpoint_to_same_result_as_uninterrupted_run():
    def sphere(x):
        return float(np.sum(x**2))

    kwargs = {
        "cost_func": sphere,
        "lower_bound": np.array([-5.0, -5.0]),
        "upper_bound": np.array([5.0, 5.0]),
        "swarm_size": 8,
        "max_iter": 9,
        "max_stall": 20,
        "parallel": False,
        "verbose": False,
        "seed": 12,
    }
    expected = ParticleSwarmOptimizer(**kwargs).optimize()
    checkpoint = {}

    def interrupt_after_two_iterations(state):
        checkpoint.update(state)
        if state["iteration"] == 2:
            raise RuntimeError("simulated interruption")

    try:
        ParticleSwarmOptimizer(**kwargs).optimize(checkpoint_callback=interrupt_after_two_iterations)
    except RuntimeError as exc:
        assert str(exc) == "simulated interruption"
    else:
        pytest.fail("expected the checkpoint callback to interrupt the PSO")

    resumed = ParticleSwarmOptimizer(**{**kwargs, "seed": 999}).optimize(resume_state=checkpoint)
    np.testing.assert_array_equal(resumed[0], expected[0])
    assert resumed[1] == expected[1]
    assert resumed[2] == expected[2]


def test_training_objective_is_arithmetic_mean_and_rejects_any_failed_condition():
    records = [
        {"cost": 2.0, "failed": False},
        {"cost": 6.0, "failed": False},
        {"cost": 10.0, "failed": False},
    ]
    aggregate = aggregate_scenario_records(records, failure_cost=999.0)
    assert aggregate["cost"] == pytest.approx(6.0)
    assert aggregate["failed"] is False

    records[1]["failed"] = True
    rejected = aggregate_scenario_records(records, failure_cost=999.0)
    assert rejected["cost"] == pytest.approx(999.0)
    assert rejected["failed"] is True


def test_optimization_context_fingerprint_tracks_training_and_pso_settings():
    first = _optimization_context(trainingConditions=[{"replayId": "r1"}], maxIterations=12)
    same = _optimization_context(trainingConditions=[{"replayId": "r1"}], maxIterations=12)
    changed = _optimization_context(trainingConditions=[{"replayId": "r2"}], maxIterations=12)
    assert first["contextHash"] == same["contextHash"]
    assert first["contextHash"] != changed["contextHash"]
    assert first["objectiveVersion"]


def test_bregman_profile_resumes_after_a_completed_grid_point(monkeypatch):
    import agc.opt.bregman_profile as profile_module

    def fake_evaluate(args):
        candidate = args[0]
        return {
            "cost": abs(float(candidate[15]) + 3.0), "failed": False,
            "candidate": candidate.copy(), "gains": {}, "label": "adaptive",
        }

    monkeypatch.setattr(profile_module, "_eval_bregman_worker", fake_evaluate)
    candidate = np.zeros(16)
    candidate[15] = -3.0
    checkpoint = {}

    def stop_after_two(state):
        checkpoint.update(state)
        if len(state["records"]) == 2:
            raise RuntimeError("pause profile")

    with pytest.raises(RuntimeError, match="pause profile"):
        profile_module.profile_bregman_gain(
            base_scenarios=[{}], incumbent_candidate=candidate,
            seed_values=[1e-3], parallel=False,
            checkpoint_callback=stop_after_two,
        )
    resumed = profile_module.profile_bregman_gain(
        base_scenarios=[{}], incumbent_candidate=candidate,
        seed_values=[1e-3], parallel=False, resume_state=checkpoint,
    )
    assert resumed["best"]["cost"] == pytest.approx(0.0)
