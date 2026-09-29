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
    assert compute_nominal_reaching_bound(run, np.eye(6), np.eye(6), 1, 1, .5) > 0


def test_recovery_includes_dwell_endpoint():
    H = np.repeat(np.eye(4)[None], 4, axis=0)
    Hd = H.copy()
    H[2, 0, 3] = 1
    run = {"t": np.arange(4.), "H": H, "Hdesired": Hd}
    assert compute_recovery_time(run, 0, dwell_time=2) is None


def test_primary_controllers_share_gains_and_initial_parameters():
    from agc.sim.publication import (paper_scenario, controller_gain_rows, COMMON_KEYS,
                                     ADAPTIVE_MODES, PRIMARY_MODES)
    from agc.sim.run_scenario import estimate_to_pi
    assert ADAPTIVE_MODES == ("euclidean", "bregman")
    assert PRIMARY_MODES == ("nominal", "euclidean", "bregman")
    scenarios = [paper_scenario(mode) for mode in PRIMARY_MODES]
    for scenario in scenarios:
        for key in COMMON_KEYS:
            np.testing.assert_array_equal(scenario["controller"][key], scenarios[0]["controller"][key])
        np.testing.assert_allclose(estimate_to_pi(scenario["controller"]["mode"], scenario["initialEstimate"]),
                                   scenarios[0]["initialEstimate"])
    saved_scenarios = dict(zip(PRIMARY_MODES, scenarios))
    assert controller_gain_rows(saved_scenarios)[0][
        "Tracking gains common across payload-release comparison?"
    ] == "yes"


def test_adaptive_payload_baseline_and_nominal_validation_are_distinct():
    from agc.sim.publication import paper_scenario
    from agc.sim.publication_runner import nominal_scenario

    payload_baseline = paper_scenario("nominal", .02)
    nominal_validation = nominal_scenario(.02)
    assert payload_baseline["payloadDrop"]["releaseTime"] == 10.0
    assert nominal_validation["payloadDrop"] is None
    assert not np.array_equal(payload_baseline["controller"]["KR"],
                              nominal_validation["controller"]["KR"])


def test_gain_report_does_not_claim_optimization_during_paper_run():
    from agc.sim.publication import gain_report

    report = gain_report()
    assert report["optimization_executed"] is False
    assert "adaptive_base" in report["adaptive_comparison_tracking_gain_source"]
    assert "separate" in report["nominal_study_tracking_gain_source"].lower()


def test_known_inertia_schedule_uses_active_plant_parameters():
    from agc.sim.run_scenario import controller_estimate
    loaded, bare = np.ones(10), np.ones(10) * 2
    cfg = {"mode": "nominal", "knownInertiaSchedule": "active-plant"}
    np.testing.assert_array_equal(controller_estimate(cfg, loaded, bare), bare)
    np.testing.assert_array_equal(controller_estimate({"mode": "nominal"}, loaded, bare), loaded)


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


def test_short_adaptive_run_has_no_post_release_physical_margin():
    from agc.sim.publication import physical_consistency_row
    H = np.eye(4)[None]
    run = {"t": np.array([0.]), "H": H, "Hdesired": H.copy(),
           "wrench": np.zeros((1, 6)), "minPseudoEigenvalue": np.array([.2]),
           "estimatePi": np.ones((1, 10))}
    row = physical_consistency_row(run, "euclidean")
    assert row["Minimum post-release lambda_min(Jhat)"] is None


def test_publication_styles_follow_paper_controller_convention():
    from agc.viz.publication_figures import STYLES, CONNECTION_STYLES
    assert STYLES["nominal"] == {"color": "#FF0000", "linestyle": "--"}
    assert STYLES["euclidean"] == {"color": "#00FF00", "linestyle": "-."}
    assert STYLES["bregman"] == {"color": "#0000FF", "linestyle": "-"}
    assert CONNECTION_STYLES["lc"] == {"color": "#0000FF", "linestyle": "-"}
    assert CONNECTION_STYLES["rb"] == {"color": "#FF0000", "linestyle": "--"}


def test_paper_summary_rows_use_final_controller_names_and_physical_labels():
    from agc.sim.publication import adaptive_performance_row, physical_consistency_row
    H = np.repeat(np.eye(4)[None], 2, axis=0)
    run = {"t": np.array([0., 10.]), "H": H, "Hdesired": H.copy(),
           "wrench": np.zeros((2, 6)), "minPseudoEigenvalue": np.array([.2, .1]),
           "estimatePi": np.ones((2, 10))}
    row = adaptive_performance_row(run, "bregman")
    assert row["Controller"] == "Natural/Bregman adaptive controller"
    assert "Minimum lambda_min(Jhat)" in row
    physical = physical_consistency_row(run, "euclidean")
    assert physical["Physical-consistency violation?"] == "no"


def test_connection_pair_protocol_requires_only_the_coriolis_choice_to_change():
    from agc.sim.publication_runner import connection_pair_protocol, connection_pair_passed
    H = np.eye(4)[None]
    V = np.zeros((1, 6))
    metadata = {"mode": "nominal", "coriolis": "lc", "payloadDrop": None,
                "plantPi": np.ones(10).tolist(), "initialEstimate": np.ones(10).tolist(),
                "initial": {"H": H[0].tolist(), "V": V[0].tolist()}, "controller": {}}
    run = {"H": H, "V": V, "Hdesired": H.copy(), "Vdesired": V.copy(), "metadata": metadata}
    common = {"KR": np.eye(3), "Kxi": np.eye(3), "Lambda": np.eye(6),
              "kd": 1., "ks": 1., "alpha": .5, "gravity": np.array([0., 0., 9.81])}
    lc = {"controller": {**common, "coriolis": "lc"}, "plantPi": np.ones(10)}
    rb = {"controller": {**common, "coriolis": "rb"}, "plantPi": np.ones(10)}
    rb_run = dict(run, metadata={**metadata, "coriolis": "rb"})
    protocol = connection_pair_protocol(run, rb_run)
    assert protocol["shared_controller_gains"]
    assert protocol["identical_desired_trajectory_samples"]
    assert connection_pair_passed(protocol)


def test_connection_protocol_rejects_adaptive_or_payload_metadata():
    from agc.sim.publication_runner import connection_pair_protocol, connection_pair_passed
    H = np.eye(4)[None]; V = np.zeros((1, 6))
    common = {"plantPi": np.ones(10).tolist(), "initialEstimate": np.ones(10).tolist(),
              "initial": {"H": H[0].tolist(), "V": V[0].tolist()}, "controller": {}, "coriolis": "lc"}
    lc = {"H": H, "V": V, "Hdesired": H.copy(), "Vdesired": V.copy(),
          "metadata": {**common, "mode": "euclidean", "payloadDrop": {"releaseTime": 10}}}
    rb = {**lc, "metadata": {**common, "mode": "nominal", "payloadDrop": None, "coriolis": "rb"}}
    protocol = connection_pair_protocol(lc, rb)
    assert protocol["adaptation"]
    assert protocol["payload_release"]
    assert not connection_pair_passed(protocol)


def test_scenario_fingerprint_changes_with_duration_and_reference():
    from agc.sim.publication_runner import scenario_fingerprint
    scenario = {"duration": 1., "dtPlant": .1, "dtControl": .1, "dtAdaptation": .1,
                "plantPi": np.ones(10), "plantGravity": np.zeros(3), "payloadDrop": None,
                "initial": {"H": np.eye(4), "V": np.zeros(6)}, "initialEstimate": np.ones(10),
                "controller": {"mode": "nominal", "coriolis": "lc"}, "replayId": "unit",
                "trajectory": lambda t: {"H": np.eye(4), "V": np.ones(6)*t, "Vdot": np.ones(6)}}
    changed_duration = {**scenario, "duration": 2.}
    changed_reference = {**scenario, "trajectory": lambda t: {"H": np.eye(4), "V": np.ones(6)*(t+1), "Vdot": np.ones(6)}}
    assert scenario_fingerprint(scenario) != scenario_fingerprint(changed_duration)
    assert scenario_fingerprint(scenario) != scenario_fingerprint(changed_reference)


def test_fresh_load_or_run_returns_saved_raw_metadata(monkeypatch, tmp_path):
    from agc.sim import publication_runner
    scenario = {"duration": 0., "dtPlant": .1, "dtControl": .1, "dtAdaptation": .1,
                "plantPi": np.ones(10), "plantGravity": np.zeros(3), "payloadDrop": None,
                "initial": {"H": np.eye(4), "V": np.zeros(6)}, "initialEstimate": np.ones(10),
                "controller": {"mode": "nominal", "coriolis": "lc"}, "replayId": "unit",
                "trajectory": lambda t: {"H": np.eye(4), "V": np.zeros(6), "Vdot": np.zeros(6)}}
    run = {"t": np.array([0.]), "H": np.eye(4)[None], "V": np.zeros((1, 6)),
           "Hdesired": np.eye(4)[None], "Vdesired": np.zeros((1, 6)), "VdotDesired": np.zeros((1, 6))}
    monkeypatch.setattr(publication_runner, "run_scenario", lambda _: (run, None))
    saved, failure, reused = publication_runner.load_or_run(tmp_path, scenario)
    assert failure is None and not reused
    assert saved["metadata"]["cache"]["scenario_sha256"] == publication_runner.scenario_fingerprint(scenario)


def test_nominal_validation_uses_the_lemniscate_at_payload_release_phase():
    from agc.sim.publication_runner import nominal_scenario
    scenario = nominal_scenario(.02)
    source = scenario["sourceTrajectory"]
    release_reference = source(10.)
    np.testing.assert_allclose(scenario["trajectory"](0.)["H"], release_reference["H"])
    np.testing.assert_allclose(scenario["trajectory"](0.)["V"], release_reference["V"])
    assert scenario["replayId"].endswith("@payload-release-phase-10s")
    assert not np.allclose(scenario["initial"]["V"], release_reference["V"])


def test_nominal_validation_never_clamps_the_replay_reference():
    from agc.sim.publication_runner import nominal_scenario
    scenario = nominal_scenario(30.)
    replay_end = scenario["sourceTrajectory"].__self__.t[-1]
    assert scenario["duration"] <= replay_end - 10.
    assert scenario["trajectory"](scenario["duration"])["H"].shape == (4, 4)


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
    assert len(list(tmp_path.rglob("*.pdf"))) == 10
    assert len(list(tmp_path.rglob("*.png"))) == 10
    fig, axes = panels(["a", "b"])
    assert axes[0].get_xlim() == (0, 30)
    plt.close(fig)


def test_connection_figure_uses_full_horizon_and_preserves_log_residual(monkeypatch, tmp_path):
    from agc.viz import publication_figures as figures
    import matplotlib.pyplot as plt

    captured = {}
    # Capture the actual plotted artists at the export boundary for inspection.
    monkeypatch.setattr(figures, "save", lambda fig, out, name: captured.update({name: fig}))
    t = np.array([0., 1., 2., 30.])
    residual = np.array([1e-14, 1e-16, 1e-17, 2e-15])
    run = {"t": t, "s": np.zeros((4, 6)), "Vs": np.zeros(4)}
    scenario = {"controller": {"Lambda": np.eye(6)}}
    summary = {"T_obs": 1.336, "T_bound": 59.75, "epsilon_s": 1e-8, "passed": True}
    connection = {"passed": True, "t": t, "difference": np.exp(-t),
                  "theory": np.exp(-t), "residual": residual}
    try:
        figures.theory_figures(run, scenario, tmp_path, summary, connection)
        axes = captured["connection_equivalence"].axes
        assert axes[0].get_xlim() == pytest.approx((0, 30))
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
        assert all(ax.get_title() == "" for ax in axes)
    finally:
        for fig in captured.values():
            plt.close(fig)


def test_connection_tracking_draws_reference_below_realizations(monkeypatch, tmp_path):
    """A desired line must not visually cover the LC/RB trajectories."""
    from agc.viz import publication_figures as figures

    captured = {}
    monkeypatch.setattr(figures, "save", lambda fig, out, name: captured.update({name: fig}))
    t = np.array([0.0, 1.0])
    H_lc = np.repeat(np.eye(4)[None], 2, axis=0)
    H_rb = H_lc.copy()
    H_lc[:, 0, 3] = [0.0, 1.0]
    H_rb[:, 0, 3] = [0.0, 1.1]
    desired = H_lc.copy()
    desired[:, 0, 3] = [0.0, 1.05]
    base = {"t": t, "Hdesired": desired, "Vdesired": np.zeros((2, 6)),
            "V": np.zeros((2, 6)), "s": np.zeros((2, 6))}
    runs = {"lc": {**base, "H": H_lc}, "rb": {**base, "H": H_rb}}
    scenarios = {key: {"controller": {"Lambda": np.eye(6)}} for key in runs}

    figures.connection_realization_figures(runs, scenarios, tmp_path)

    labels = [line.get_label() for line in captured["connection_tracking_position"].axes[0].lines]
    assert labels[:3] == ["Desired reference", r"$C_{\mathrm{LC}}$", r"$C_{\mathrm{RB}}$"]


def test_connection_identity_uses_fixed_full_horizon(monkeypatch, tmp_path):
    from agc.viz import publication_figures as figures

    captured = {}
    monkeypatch.setattr(figures, "save", lambda fig, out, name: captured.update({name: fig}))
    t = np.array([0.0, 1.0, 2.0, 30.0])
    run = {"t": t, "s": np.zeros((4, 6)), "Vs": np.zeros(4)}
    scenario = {"controller": {"Lambda": np.eye(6)}}
    summary = {"T_obs": 1.0, "T_bound": 59.75, "epsilon_s": 1e-8, "passed": True}
    connection = {"passed": True, "t": t, "difference": np.array([.2, .1, .05, 2.0]),
                  "theory": np.array([.2, .1, .05, 2.0]), "residual": np.full(4, 1e-14)}

    figures.theory_figures(run, scenario, tmp_path, summary, connection)

    assert captured["connection_equivalence"].axes[0].get_xlim() == pytest.approx((0, 30))


def test_connection_identity_uses_lemniscate_source_time_when_available(monkeypatch, tmp_path):
    from agc.viz import publication_figures as figures

    captured = {}
    monkeypatch.setattr(figures, "save", lambda fig, out, name: captured.update({name: fig}))
    t = np.array([0.0, 1.0, 2.0])
    run = {"t": t, "s": np.zeros((3, 6)), "Vs": np.zeros(3)}
    scenario = {"controller": {"Lambda": np.eye(6)}, "timeOffset": 10.0}
    summary = {"T_obs": 1.0, "T_bound": 59.75, "epsilon_s": 1e-8, "passed": True}
    connection = {"passed": True, "t": t, "difference": np.array([.2, .1, .05]),
                  "theory": np.array([.2, .1, .05]), "residual": np.full(3, 1e-14)}

    figures.theory_figures(run, scenario, tmp_path, summary, connection)

    axes = captured["connection_equivalence"].axes
    assert axes[0].get_xlim() == pytest.approx((10.0, 30.0))
    np.testing.assert_allclose(axes[0].lines[-1].get_xdata(), [11.0, 11.0])
    assert axes[1].get_xlabel() == "Lemniscate time [s]"


def test_nominal_figures_use_the_paper_time_window(monkeypatch, tmp_path):
    from agc.viz import publication_figures as figures

    captured = {}
    monkeypatch.setattr(figures, "save", lambda fig, out, name: captured.update({name: fig}))
    t = np.array([0.0, 20.0, 20.882])
    H = np.repeat(np.eye(4)[None], len(t), axis=0)
    run = {"t": t, "H": H, "Hdesired": H, "V": np.zeros((len(t), 6)),
           "Vdesired": np.zeros((len(t), 6)), "s": np.zeros((len(t), 6)), "Vs": np.zeros(len(t))}
    scenarios = {key: {"controller": {"Lambda": np.eye(6)}, "timeOffset": 10.0,
                       "displayEnd": 30.0} for key in ("lc", "rb")}
    summary = {"T_obs": 1.0, "T_bound": 59.75, "epsilon_s": 1e-8, "passed": True}
    connection = {"passed": True, "t": t, "difference": np.array([.2, .1, .05]),
                  "theory": np.array([.2, .1, .05]), "residual": np.full(3, 1e-14)}

    figures.connection_realization_figures({"lc": run, "rb": run}, scenarios, tmp_path)
    figures.theory_figures(run, scenarios["lc"], tmp_path, summary, connection)

    position_axes = captured["connection_tracking_position"].axes
    assert position_axes[0].get_xlim() == pytest.approx((10.0, 30.0))
    np.testing.assert_allclose(position_axes[0].lines[0].get_xdata(), [10.0, 30.0])
    comparison_axes = captured["connection_realization_comparison"].axes
    for ax in comparison_axes:
        for line in ax.lines:
            np.testing.assert_allclose(line.get_xdata(), [10.0, 30.0])
    reaching_axes = captured["nominal_reaching"].axes
    assert reaching_axes[0].get_xlim() == pytest.approx((10.0, 30.0))
    assert reaching_axes[0].get_yscale() == "symlog"
    assert any("59.75" in text.get_text() for text in reaching_axes[0].texts)


def test_connection_identity_shows_one_signal_and_its_residual(monkeypatch, tmp_path):
    from agc.viz import publication_figures as figures

    captured = {}
    monkeypatch.setattr(figures, "save", lambda fig, out, name: captured.update({name: fig}))
    t = np.array([0.0, 1.0])
    run = {"t": t, "s": np.zeros((2, 6)), "Vs": np.zeros(2)}
    scenario = {"controller": {"Lambda": np.eye(6)}}
    summary = {"T_obs": None, "T_bound": 1.0, "epsilon_s": 1e-8, "passed": True}
    connection = {"passed": True, "t": t, "difference": np.array([.2, .1]),
                  "theory": np.array([.2, .1]), "residual": np.full(2, 1e-14)}

    figures.theory_figures(run, scenario, tmp_path, summary, connection)

    labels = [line.get_label() for line in captured["connection_equivalence"].axes[0].lines]
    assert labels == [r"$\|\mathcal{W}_c^{RB}-\mathcal{W}_c^{LC}\|_*$"]
