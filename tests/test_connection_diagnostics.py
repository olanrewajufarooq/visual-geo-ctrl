import numpy as np

from agc.paper.diagnostics import connection_identity
from agc.sim.default_scenario import default_scenario


def test_connection_identity_matches_transverse_term():
    scenario = default_scenario(mode="nominal", coriolis="lc", duration=0.02, payload_enabled=False)
    state = scenario["initial"]
    desired = scenario["trajectory"](0.0)

    result = connection_identity(state, desired, scenario["controller"], scenario["plantPi"])

    assert result["residualNorm"] < 1e-10
