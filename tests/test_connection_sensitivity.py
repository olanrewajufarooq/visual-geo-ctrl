import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from agc.sim.connection_sensitivity import focus_end, separation, sensitivity_scenario


def test_focus_includes_later_dwell_and_rounds_up():
    assert focus_end([1., 1.336], 20.) == 2.25
    assert focus_end([None, 1.], 20.) == 20.
    assert focus_end([19.5, 19.6], 20.) == 20.


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
