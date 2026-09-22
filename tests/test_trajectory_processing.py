"""Tests for trajectory processing, SE(3) smoother, and .npz artifact replay."""

import json
import subprocess
import sys
from unittest.mock import patch
from pathlib import Path
import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))
sys.path.insert(0, str(REPO_ROOT))

from agc.math.se3 import exp_se3, log_se3, log_so3, expm_so3
from agc.sim.replay_trajectory import ReplayTrajectory
from trajectories.replay_scripts import (
    ReplayProcessor,
    ReplayProcessingCore,
    ReplayKinematics,
    ReplayWnojSmoother,
    ReplayTraj,
    write_replay_artifact,
)


def test_trajectory_preprocessing_cli_bootstraps_the_agc_package():
    """The documented preprocessing CLI must run without an installed package."""
    root = Path(__file__).resolve().parent.parent
    result = subprocess.run(
        [sys.executable, str(root / "trajectories" / "process_trajectories.py"), "--help"],
        cwd=root,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
    assert "Build canonical SE(3) replay artifacts" in result.stdout


def test_replay_processing_applies_cli_postprocessing_overrides(tmp_path):
    """Routine preprocessing must pass its bounded WNOJ settings to each replay."""
    manifest = {
        "test_replay": {
            "source_file": "source.csv",
            "artifact_file": "test_replay.npz",
        }
    }
    manifest_path = tmp_path / "manifest.json"
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    (tmp_path / "source.csv").write_text("placeholder", encoding="utf-8")

    with patch.object(ReplayKinematics, "process_single", return_value={}) as process_single:
        with patch("trajectories.replay_scripts.replay_processing_core.write_replay_artifact"):
            ReplayProcessingCore.process_all(
                root_dir=tmp_path,
                manifest_path=manifest_path,
                trajectory_ids=["test_replay"],
                postprocessing_options={"knotIntervalSeconds": 0.25, "maxIterations": 10},
            )

    entry = process_single.call_args.args[2]
    assert entry["postprocessing"]["wnoj"] == {"knotIntervalSeconds": 0.25, "maxIterations": 10}


def test_se3_exp_log_roundtrip():
    """Verify SE(3) exponential and logarithmic maps are exact inverses."""
    # Test small perturbation
    xi_small = np.array([1e-4, -2e-4, 3e-4, 0.05, -0.02, 0.01])
    H_small = exp_se3(xi_small)
    xi_small_rec = log_se3(H_small)
    assert np.allclose(xi_small, xi_small_rec, atol=1e-12)

    # Test larger twist
    xi_large = np.array([0.5, -0.3, 0.8, 2.5, -1.2, 4.0])
    H_large = exp_se3(xi_large)
    xi_large_rec = log_se3(H_large)
    assert np.allclose(xi_large, xi_large_rec, atol=1e-10)

    # Test pure translation
    xi_trans = np.array([0.0, 0.0, 0.0, 1.0, 2.0, 3.0])
    H_trans = exp_se3(xi_trans)
    assert np.allclose(H_trans[0:3, 0:3], np.eye(3))
    assert np.allclose(H_trans[0:3, 3], [1.0, 2.0, 3.0])
    assert np.allclose(log_se3(H_trans), xi_trans)

    # Test pure rotation
    xi_rot = np.array([0.2, -0.4, 0.1, 0.0, 0.0, 0.0])
    H_rot = exp_se3(xi_rot)
    assert np.allclose(H_rot[0:3, 3], np.zeros(3))
    assert np.allclose(log_se3(H_rot), xi_rot)


def test_npz_canonical_artifacts_load_and_sample():
    """Verify all 3 canonical .npz artifacts load and sample cleanly."""
    root = Path(__file__).resolve().parent.parent
    processed_dir = root / "trajectories" / "processed"

    for traj_id in ["ellipse_01_auto", "lemniscate_01_auto", "RATM_01_auto"]:
        npz_file = processed_dir / f"{traj_id}.npz"
        assert npz_file.is_file(), f"Missing canonical .npz artifact: {npz_file}"

        sampler = ReplayTrajectory(str(npz_file))
        assert len(sampler.t) > 1000
        assert sampler.t[0] == 0.0
        assert sampler.t[-1] > 10.0

        # Sample at t = 0.0, midpoint, and end
        for t_query in [0.0, float(sampler.t[-1] / 2.0), float(sampler.t[-1])]:
            sample = sampler.sample(t_query)
            assert "H" in sample and "V" in sample and "Vdot" in sample

            H = sample["H"]
            V = sample["V"]
            Vdot = sample["Vdot"]

            assert H.shape == (4, 4)
            assert V.shape == (6,)
            assert Vdot.shape == (6,)
            assert np.all(np.isfinite(H))
            assert np.all(np.isfinite(V))
            assert np.all(np.isfinite(Vdot))

            # Rotation orthogonality
            R = H[0:3, 0:3]
            assert np.allclose(R.T @ R, np.eye(3), atol=1e-4)
            assert np.isclose(np.linalg.det(R), 1.0, atol=1e-4)


def test_replay_processor_facade():
    """Verify public facade ReplayProcessor methods."""
    root_dir = ReplayProcessor.default_root_dir()
    assert root_dir.is_dir()

    entry = ReplayProcessor.load_entry("lemniscate_01_auto")
    assert entry["id"] == "lemniscate_01_auto"
    assert Path(entry["artifact_path"]).is_file()

    traj = ReplayProcessor.load_artifact("lemniscate_01_auto")
    assert "t" in traj and "p" in traj and "R" in traj

    limits = ReplayProcessor.default_axis_limits("lemniscate_01_auto", pad=1.0)
    assert len(limits) == 6
    assert limits[0] < limits[1]  # xmin < xmax
    assert limits[2] < limits[3]  # ymin < ymax


def test_replay_traj_class():
    """Verify ReplayTraj wrapper generates H, V, A states."""
    ref = ReplayTraj("lemniscate_01_auto")
    H, V, A = ref.generate(1.5)
    assert H.shape == (4, 4)
    assert V.shape == (6,)
    assert A.shape == (6,)

    # Dict call
    sample = ref(1.5)
    assert np.allclose(sample["H"], H)
    assert np.allclose(sample["V"], V)
    assert np.allclose(sample["Vdot"], A)


def test_write_and_reload_replay_artifact(tmp_path):
    """Verify writing and reloading a processed .npz trajectory."""
    t = np.linspace(0.0, 1.0, 100)
    p = np.zeros((100, 3))
    p[:, 0] = np.sin(t)
    v_b = np.zeros((100, 3))
    a_b = np.zeros((100, 3))
    omega_b = np.zeros((100, 3))
    alpha_b = np.zeros((100, 3))
    R = np.zeros((3, 3, 100))
    for i in range(100):
        R[:, :, i] = np.eye(3)

    traj = {
        "t": t,
        "p": p,
        "v_b": v_b,
        "a_b": a_b,
        "omega_b": omega_b,
        "alpha_b": alpha_b,
        "R": R,
        "meta": {"id": "synthetic_test", "sampleRateHz": 100.0},
    }

    out_file = tmp_path / "synthetic_test.npz"
    saved = write_replay_artifact(out_file, traj, output_format="npz")
    assert saved.is_file()

    sampler = ReplayTrajectory(str(saved))
    assert len(sampler.t) == 100
    assert np.allclose(sampler.p, p)
    sample = sampler.sample(0.5)
    assert np.allclose(sample["H"][0:3, 0:3], np.eye(3))


def test_wnoj_batch_smoother_nonlinear_optimization():
    """Verify full nonlinear SE(3) WNOJ batch smoother with LM optimization."""
    t = np.linspace(0.0, 1.0, 50)
    p = np.zeros((50, 3))
    p[:, 0] = np.sin(2.0 * np.pi * t)
    p[:, 1] = np.cos(2.0 * np.pi * t)
    p[:, 2] = 0.5 * t

    R = np.zeros((3, 3, 50))
    for i in range(50):
        R[:, :, i] = np.eye(3)

    v_b = np.zeros((50, 3))
    v_b[:, 0] = 2.0 * np.pi * np.cos(2.0 * np.pi * t)
    v_b[:, 1] = -2.0 * np.pi * np.sin(2.0 * np.pi * t)
    v_b[:, 2] = 0.5

    omega_b = np.zeros((50, 3))
    V_body = np.hstack([omega_b, v_b])

    smoother = ReplayWnojSmoother({
        "knotIntervalSeconds": 0.05,
        "maxIterations": 50,
        "requireConvergence": True,
    })
    smoother.fit(t, R, p, V_body)

    assert smoother.diagnostics["converged"] is True
    assert smoother.diagnostics["finalCost"] < smoother.diagnostics["initialCost"]
    assert smoother.diagnostics["iterations"] > 0
    assert len(smoother.t) >= 10

    # Evaluate continuous SE(3) state
    H, V, A = smoother.evaluate(0.33)
    assert H.shape == (4, 4)
    assert V.shape == (6,)
    assert A.shape == (6,)
    assert np.allclose(H[0:3, 0:3].T @ H[0:3, 0:3], np.eye(3), atol=1e-4)

    # Validate internal kinematic consistency
    residuals = smoother.validate()
    assert np.isfinite(residuals["maxPoseTwistResidual"])
    assert np.isfinite(residuals["maxTwistAccelerationResidual"])
