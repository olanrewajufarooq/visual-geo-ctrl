import numpy as np

from agc.math.se3 import adjoint_se3, inv_se3
from agc.sim.metrics import compute_metrics


def test_velocity_error_uses_transported_desired_twist_and_split_wrench_units():
    hd = np.eye(4)
    hd[:3, :3] = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    hd[:3, 3] = [1.0, 2.0, 0.0]
    h = np.eye(4)
    h[:3, 3] = [3.0, 2.0, 0.0]
    vd = np.array([0.1, -0.2, 0.3, 1.0, 2.0, -1.0])
    he = inv_se3(hd) @ h
    v = adjoint_se3(inv_se3(he)) @ vd
    run = {
        "t": np.array([0.0]), "H": h[None], "Hdesired": hd[None],
        "V": v[None], "Vdesired": vd[None], "wrench": np.array([[3., 4., 0., 0., 0., 12.]]),
        "s": np.zeros((1, 6)), "Psi": np.zeros(1), "Vs": np.zeros(1),
        "minPseudoEigenvalue": np.array([np.nan]),
    }
    metrics = compute_metrics(run)
    assert metrics["angularVelocityRMSE"] < 1e-12
    assert metrics["linearVelocityRMSE"] < 1e-12
    assert metrics["forceRMS"] == 12.0
    assert metrics["torqueRMS"] == 5.0
