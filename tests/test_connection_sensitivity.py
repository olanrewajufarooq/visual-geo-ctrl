import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from agc.sim.connection_sensitivity import separation, sensitivity_scenario


def test_connections_use_independent_persistent_reaching_times():
    from agc.sim.connection_sensitivity import audit
    from agc.sim.publication_runner import nominal_scenario
    scenario = nominal_scenario(20.)
    runs = []
    for settle in (2., 5.):
        t = np.arange(0., 20.001, .1)
        s = np.zeros((len(t), 6)); s[t < settle, 0] = 1.
        H = np.repeat(np.eye(4)[None], len(t), axis=0)
        runs.append({'t':t, 's':s, 'H':H, 'V':np.zeros((len(t),6)),
                     'Hdesired':H, 'Vdesired':np.zeros((len(t),6)),
                     'VdotDesired':np.zeros((len(t),6)), 'wrench':np.zeros((len(t),6))})
    a, b = audit(runs[0], scenario), audit(runs[1], scenario)
    assert a['persistent_reaching_elapsed_s'] != b['persistent_reaching_elapsed_s']
    assert a['persistent_reaching_absolute_s'] == pytest.approx(a['persistent_reaching_elapsed_s'] + 10.)
    assert a['persistent_threshold'] == 1e-8
    assert a['persistent_final_source_time_s'] == 30.


def test_separation_uses_physical_units_and_common_axes():
    a = {'t': np.array([0.]), 'H': np.eye(4)[None], 'wrench': np.array([[1., 0., 0., 1., 0., 0.]])}
    b = {k: v.copy() for k, v in a.items()}
    b['H'][0, :3, :3] = Rotation.from_euler('z', 90, degrees=True).as_matrix()
    b['H'][0, :3, 3] = [3., 4., 0.]
    b['wrench'][0] = [0., -1., 0., 0., -1., 0.]
    values = separation(a, b)
    assert values['Position separation [m]'][0] == pytest.approx(5.)
    assert values['Attitude separation [deg]'][0] == pytest.approx(90.)
    assert values['Force difference [N]'][0] < 1e-14
    assert values['Torque difference [N m]'][0] < 1e-14


def test_sensitivity_pair_only_changes_connection():
    a, b = sensitivity_scenario('lc'), sensitivity_scenario('rb')
    np.testing.assert_allclose(a['initial']['V']-a['trajectory'](0)['V'], [1.2, -.8, .4, 1.6, -.8, 1.2])
    assert a['payloadDrop'] is None
    assert a['duration'] == 20.
    for key in a['controller']:
        if key != 'coriolis':
            np.testing.assert_equal(a['controller'][key], b['controller'][key])


@pytest.mark.parametrize('command', ['nominal-reaching', 'nominal-connection', 'connection-realizations', 'connection-sensitivity'])
def test_all_nominal_commands_use_sensitivity(command, monkeypatch, tmp_path):
    from agc.sim import connection_sensitivity, publication_runner
    calls = []
    monkeypatch.setattr(connection_sensitivity, 'run_study', lambda root, raw: calls.append((root, raw)))
    publication_runner.run_publication(command, 30., tmp_path, tmp_path/'raw')
    assert calls == [(tmp_path, tmp_path/'raw')]
