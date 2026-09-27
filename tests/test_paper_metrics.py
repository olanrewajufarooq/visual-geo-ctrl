import numpy as np
import pytest

from agc.sim.paper_metrics import (
    compute_nominal_reaching_bound,
    compute_reaching_time,
    compute_persistent_reaching_time,
    compute_recovery_time,
    compute_window_metrics,
)


def _run():
    t = np.arange(0.0, 5.0, 0.1)
    H = np.repeat(np.eye(4)[None, :, :], len(t), axis=0)
    Hd = H.copy()
    H[:, 0, 3] = np.where(t < 2.0, 0.1, 0.0)
    s = np.zeros((len(t), 6))
    s[:, 0] = np.where(t < 1.0, 2.0, 0.0)
    return {
        "t": t,
        "H": H,
        "Hdesired": Hd,
        "wrench": np.ones((len(t), 6)),
        "s": s,
        "Psi": np.ones(len(t)),
        "Vs": np.ones(len(t)),
    }


def test_window_metrics_and_recovery_use_intrinsic_errors():
    run = _run()
    metrics = compute_window_metrics(run, 0.0, 1.0)
    assert metrics["positionRMSE"] == pytest.approx(0.1)
    assert metrics["attitudeRMSE"] == pytest.approx(0.0)
    assert metrics["peakForce"] == pytest.approx(np.sqrt(3.0))
    assert metrics["peakTorque"] == pytest.approx(np.sqrt(3.0))
    assert metrics["forceRMS"] == pytest.approx(np.sqrt(3.0))
    assert metrics["torqueRMS"] == pytest.approx(np.sqrt(3.0))
    assert "wrenchRMS" not in metrics
    assert compute_recovery_time(run, 0.0, dwell_time=1.0) == pytest.approx(2.0)


def test_reaching_bound_is_zero_for_zero_initial_energy():
    run = _run()
    run["Vs"][0] = 0.0
    run["s"][0] = 0.0
    bound = compute_nominal_reaching_bound(run, np.eye(6), np.eye(6), 1.0, 1.0, 0.5)
    assert bound == 0.0


def test_nominal_bound_uses_full_kd_and_ks_comparison():
    run = _run()
    inertia = np.eye(6)
    metric = np.eye(6)
    kd, ks, alpha = 2.0, 1.0, 0.5
    q = (1 + alpha) / 2
    c = 2.0
    v0 = .5 * run['s'][0] @ inertia @ run['s'][0]
    a, b = kd*c, ks*c**q
    expected = np.log1p((a/b)*v0**(1-q))/(a*(1-q))
    assert compute_nominal_reaching_bound(run, inertia, metric, kd, ks, alpha) == pytest.approx(expected)


def test_persistent_reaching_requires_threshold_through_final_sample():
    run = _run()
    run['t'] = np.arange(0., 3.1, .1)
    run['s'] = np.zeros((len(run['t']), 6))
    run['s'][:5, 0] = 1.
    run['s'][-1, 0] = 2e-8
    assert compute_persistent_reaching_time(run, 1e-8, np.eye(6), final_time=3.) is None
    run['s'][-2, 0] = 2e-8
    run['s'][-1, 0] = 0.
    assert compute_persistent_reaching_time(run, 1e-8, np.eye(6), final_time=3.) == pytest.approx(3.)


def test_persistent_reaching_requires_complete_horizon_and_source_offset():
    run = _run()
    run['t'] = np.arange(0., 2.1, .1)
    run['s'] = np.zeros((len(run['t']), 6))
    assert compute_persistent_reaching_time(run, 1e-8, np.eye(6), final_time=30., time_offset=10.) is None


def test_reaching_time_requires_dwell():
    run = _run()
    run["s"][:, 0] = 0.0
    run["s"][3, 0] = 1.0

    assert compute_reaching_time(run, threshold=1e-3, dwell_time=0.5) == pytest.approx(0.4)
