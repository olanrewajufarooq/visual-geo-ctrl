import numpy as np
import pytest

from agc.paper.diagnostics import connection_identity
from agc.sim.default_scenario import default_scenario
from agc.sim.paper_metrics import compute_nominal_reaching_bound, compute_recovery_time


def test_connection_identity_off_manifold():
    scenario = default_scenario(payload_enabled=False)
    state = {"H": scenario["initial"]["H"].copy(), "V": np.arange(6.) / 10}
    state["H"][0, 3] += 0.2
    result = connection_identity(state, scenario["trajectory"](0), scenario["controller"], scenario["plantPi"])
    assert result["differenceNorm"] > 1e-5
    assert result["residualNorm"] < 1e-10


def test_bound_uses_true_initial_energy_not_logged_estimate():
    run = {"s": np.ones((1, 6)), "Vs": np.zeros(1)}
    assert compute_nominal_reaching_bound(run, np.eye(6), np.eye(6), 1, .5) > 0


def test_recovery_includes_dwell_endpoint():
    H = np.repeat(np.eye(4)[None], 4, axis=0)
    Hd = H.copy()
    H[2, 0, 3] = 1
    run = {"t": np.arange(4.), "H": H, "Hdesired": Hd}
    assert compute_recovery_time(run, 0, dwell_time=2) is None


def test_primary_controllers_share_gains_and_initial_parameters():
    from agc.sim.publication import paper_scenario, COMMON_KEYS, ADAPTIVE_MODES
    from agc.sim.run_scenario import estimate_to_pi
    assert ADAPTIVE_MODES == ("euclidean", "bregman")
    scenarios = [paper_scenario(mode) for mode in ADAPTIVE_MODES]
    for scenario in scenarios:
        for key in COMMON_KEYS:
            np.testing.assert_array_equal(scenario["controller"][key], scenarios[0]["controller"][key])
        np.testing.assert_allclose(estimate_to_pi(scenario["controller"]["mode"], scenario["initialEstimate"]),
                                   scenarios[0]["initialEstimate"])


def test_actual_pybullet_inertia_matches_requested_tensor():
    from agc.plant.pybullet_plant import PyBulletPlant, p
    from agc.math.inertia import inertia_from_pi
    from agc.math.se3 import quat_to_rotm
    scenario = default_scenario(payload_enabled=False)
    pi = scenario["plantPi"]
    plant = PyBulletPlant(pi)
    try:
        info = p.getDynamicsInfo(plant.uav_id, -1, physicsClientId=plant.client_id)
        R = quat_to_rotm(np.array(info[4]))
        actual = R @ np.diag(info[2]) @ R.T
        c = pi[1:4]/pi[0]
        expected = inertia_from_pi(pi)[:3, :3] - pi[0]*((c@c)*np.eye(3)-np.outer(c, c))
        np.testing.assert_allclose(actual, expected, rtol=1e-5, atol=1e-7)
    finally:
        plant.close()


def test_body_origin_wrench_matches_rigid_body_acceleration():
    from agc.plant.pybullet_plant import PyBulletPlant
    from agc.math.inertia import inertia_from_pi
    from scipy.spatial.transform import Rotation
    pi = np.array([2., .04, -.06, .02, .1, .12, .14, .003, -.002, .001])
    dt = 1e-5
    H = np.eye(4)
    H[:3, :3] = Rotation.from_euler("xyz", [.3, -.2, .4]).as_matrix()
    H[:3, 3] = [1, 2, 3]
    wrench = np.array([.1, -.2, .3, 1., -2., 3.])
    plant = PyBulletPlant(pi, gravity=np.zeros(3), dt=dt)
    try:
        plant.set_state(H, np.zeros(6))
        np.testing.assert_allclose(plant.get_state()["H"], H, atol=1e-12)
        plant.apply_wrench(wrench); plant.step()
        acceleration = plant.get_state()["V"]/dt
        np.testing.assert_allclose(acceleration, np.linalg.solve(inertia_from_pi(pi), wrench), rtol=2e-4, atol=2e-4)
    finally:
        plant.close()


def test_baseline_metrics_release_boundary_degrees_and_separate_wrench_units():
    from agc.sim.publication import baseline_metrics
    from scipy.spatial.transform import Rotation
    H = np.repeat(np.eye(4)[None], 4, axis=0)
    Hd = H.copy()
    H[:, 0, 3] = [1., 1., 2., 2.]
    H[2:, :3, :3] = Rotation.from_euler("x", 30, degrees=True).as_matrix()
    run = {"t": np.array([0., 9., 10., 11.]), "H": H, "Hdesired": Hd,
           "wrench": np.tile([3., 0, 0, 0, 4., 0], (4, 1)),
           "minPseudoEigenvalue": np.array([.2, .1, -.1, .05]),
           "estimatePi": np.ones((4, 10))}
    row = baseline_metrics(run, "euclidean")
    assert row["Pre-release position RMSE [m]"] == 1
    assert row["Post-release position RMSE [m]"] == 2
    assert row["Post-release attitude RMSE [deg]"] == pytest.approx(30)
    assert row["RMS force [N]"] == 4
    assert row["RMS torque [N m]"] == 3
    assert row["Nonpositive pseudo-inertia eigenvalue"]
    assert row["Absolute recovery time [s]"] is None


def test_weighted_reaching_does_not_use_euclidean_norm():
    from agc.sim.paper_metrics import compute_reaching_time
    run = {"t": np.arange(4.), "s": np.ones((4, 6)) * .0001}
    assert compute_reaching_time(run, lambda_s=np.eye(6)*100) is None


def test_velocity_reference_uses_full_adjoint_transport():
    from agc.viz.publication_figures import transported_reference
    from agc.math.se3 import adjoint_se3, inv_se3
    H = np.eye(4); H[:3, 3] = [1., 2., 3.]
    desired = np.array([.1, .2, .3, 1., 0, 0])
    run = {"H": H[None], "Hdesired": np.eye(4)[None], "Vdesired": desired[None]}
    np.testing.assert_allclose(transported_reference(run)[0], adjoint_se3(inv_se3(H)) @ desired)


def test_publication_exports_pdf_and_png_with_shared_limits(tmp_path):
    from agc.sim.publication import paper_scenario
    from agc.sim.run_scenario import run_scenario
    from agc.viz.publication_figures import adaptive_figures, panels
    import matplotlib.pyplot as plt
    scenarios = {mode: paper_scenario(mode, .02) for mode in ("nominal", "euclidean", "bregman")}
    runs = {}
    for mode, scenario in scenarios.items():
        runs[mode], failure = run_scenario(scenario)
        assert failure is None
    adaptive_figures(runs, scenarios, tmp_path)
    assert len(list(tmp_path.rglob("*.pdf"))) == 8
    assert len(list(tmp_path.rglob("*.png"))) == 8
    fig, axes = panels(["a", "b"])
    assert axes[0].get_xlim() == (0, 30)
    plt.close(fig)


def test_connection_figure_focuses_transient_and_preserves_log_residual(monkeypatch, tmp_path):
    from agc.viz import publication_figures as figures
    import matplotlib.pyplot as plt

    captured = {}
    # Capture the actual plotted artists at the export boundary for inspection.
    monkeypatch.setattr(figures, "save", lambda fig, out, name: captured.update({name: fig}))
    t = np.array([0., 1., 2., 30.])
    residual = np.array([1e-14, 1e-16, 1e-17, 2e-15])
    run = {"t": t, "s": np.zeros((4, 6)), "Vs": np.zeros(4)}
    scenario = {"controller": {"Lambda": np.eye(6)}}
    summary = {"T_obs": 1.336, "T_bound": 59.75, "epsilon_s": .001, "passed": True}
    connection = {"passed": True, "t": t, "difference": np.exp(-t),
                  "theory": np.exp(-t), "residual": residual}
    try:
        figures.theory_figures(run, scenario, tmp_path, summary, connection)
        axes = captured["connection_equivalence"].axes
        assert axes[0].get_xlim() == pytest.approx((0, 1.6*summary["T_obs"]))
        assert axes[1].get_xlabel() == r"Time since initialization, $t$ [s]"
        assert axes[1].get_yscale() == "log"
        np.testing.assert_array_equal(axes[1].lines[0].get_ydata(), residual)
        fig = captured["connection_equivalence"]
        fig.canvas.draw()
        renderer = fig.canvas.get_renderer()
        labels = [ax.yaxis.label.get_window_extent(renderer) for ax in axes]
        assert labels[1].y1 + 8 < labels[0].y0, "Y-axis titles must have a visible gap"
        for ax in axes:
            markers = [line for line in ax.lines if line.get_label() == r"$T_{\mathrm{obs}}$"]
            assert len(markers) == 1
            np.testing.assert_allclose(markers[0].get_xdata(), [1.336, 1.336])
    finally:
        for fig in captured.values():
            plt.close(fig)
