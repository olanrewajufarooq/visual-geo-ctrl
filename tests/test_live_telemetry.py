"""Tests for low-overhead live visualization telemetry."""

import numpy as np

from agc.viz.live_telemetry import (
    LiveTelemetryBuffer,
    VisualizationSnapshot,
    make_snapshot,
)
from agc.math.se3 import expm_so3
from agc.viz.live_dashboard import _center_of_mass


def _snapshot(t: float) -> VisualizationSnapshot:
    return make_snapshot(
        t=t,
        actual_H=np.eye(4),
        desired_H=np.eye(4),
        actual_V=np.zeros(6),
        desired_V=np.ones(6),
        sliding=np.arange(6, dtype=float),
        wrench=np.arange(6, dtype=float) + 10.0,
        estimate_pi=np.arange(10, dtype=float),
        true_pi=np.arange(10, dtype=float) + 1.0,
        payload_dropped=False,
        min_pseudo_eigenvalue=0.5,
    )


def test_make_snapshot_derives_tracking_and_parameter_signals():
    actual = np.eye(4)
    actual[:3, 3] = [1.0, 2.0, 3.0]
    desired = np.eye(4)
    desired[:3, 3] = [0.0, 1.0, 1.0]

    snapshot = make_snapshot(
        t=1.25,
        actual_H=actual,
        desired_H=desired,
        actual_V=np.array([1, 2, 3, 4, 5, 6], dtype=float),
        desired_V=np.zeros(6),
        sliding=np.arange(6, dtype=float),
        wrench=np.ones(6),
        estimate_pi=np.arange(10, dtype=float),
        true_pi=np.arange(10, dtype=float) + 1.0,
        payload_dropped=True,
        min_pseudo_eigenvalue=0.25,
    )

    np.testing.assert_allclose(snapshot.position_error, [1.0, 1.0, 2.0])
    np.testing.assert_allclose(snapshot.attitude_error, [0.0, 0.0, 0.0])
    np.testing.assert_allclose(snapshot.velocity_error, [1, 2, 3, 4, 5, 6])
    assert snapshot.position_error_norm == np.sqrt(6.0)
    assert snapshot.sliding_norm == np.sqrt(55.0)
    assert snapshot.payload_dropped is True
    assert snapshot.estimate_pi.shape == (10,)


def test_snapshot_copies_inputs_and_is_immutable_at_boundary():
    actual_v = np.ones(6)
    snapshot = make_snapshot(
        t=0.0,
        actual_H=np.eye(4),
        desired_H=np.eye(4),
        actual_V=actual_v,
        desired_V=np.zeros(6),
        sliding=np.zeros(6),
        wrench=np.zeros(6),
        estimate_pi=np.zeros(10),
        true_pi=np.ones(10),
        payload_dropped=False,
        min_pseudo_eigenvalue=np.nan,
    )

    actual_v[0] = 99.0
    assert snapshot.actual_V[0] == 1.0


def test_snapshot_uses_geometric_attitude_error():
    actual = np.eye(4)
    actual[:3, :3] = expm_so3(np.array([0.1, -0.2, 0.3]))
    snapshot = make_snapshot(
        t=0.0,
        actual_H=actual,
        desired_H=np.eye(4),
        actual_V=np.zeros(6),
        desired_V=np.zeros(6),
        sliding=np.zeros(6),
        wrench=np.zeros(6),
        estimate_pi=np.zeros(10),
        true_pi=np.ones(10),
        payload_dropped=False,
        min_pseudo_eigenvalue=np.nan,
    )

    np.testing.assert_allclose(snapshot.attitude_error, [0.1, -0.2, 0.3], atol=1e-10)


def test_dashboard_converts_first_moments_to_center_of_mass():
    parameters = np.array([[2.0, 0.2, -0.4, 0.6] + [0.0] * 6])
    np.testing.assert_allclose(_center_of_mass(parameters, np), [[0.1, -0.2, 0.3]])


def test_buffer_coalesces_to_latest_snapshot_when_full():
    buffer = LiveTelemetryBuffer(maxsize=2)
    first = _snapshot(1.0)
    second = _snapshot(2.0)
    third = _snapshot(3.0)

    assert buffer.publish(first) is True
    assert buffer.publish(second) is True
    assert buffer.publish(third) is True
    assert buffer.dropped_count == 1
    latest = buffer.get_latest()
    assert latest.t == 3.0
    assert latest is third


def test_buffer_returns_none_when_empty():
    buffer = LiveTelemetryBuffer(maxsize=1)
    assert buffer.get_latest() is None
