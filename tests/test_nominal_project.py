import numpy as np
import pytest

from vgc.config.optimized_gains import optimized_gains
from vgc.sim.default_scenario import default_scenario
from vgc.sim.validation import validate_scenario


def test_default_scenario_is_nominal_bare_vehicle():
    scenario = default_scenario(duration=0.02)

    assert scenario["controller"]["mode"] == "nominal"
    assert set(scenario["controller"]) >= {"plantPi", "KR", "Kxi", "Lambda"}
    assert set(scenario) == {
        "plantPi", "initial", "trajectory", "duration", "dtPlant", "dtControl",
        "plantGravity", "controller", "gui", "simSpeed", "enablePacing", "replayId",
        "groundZ", "groundStyle", "gatesMode", "camMode", "enableOsd", "droneType",
    }
    assert set(scenario["controller"]) == {
        "mode", "coriolis", "plantPi", "KR", "Kxi", "Lambda", "Lambda_s", "kd", "ks", "alpha", "gravity"
    }


def test_only_nominal_gain_sets_are_available():
    assert set(optimized_gains("nominal", "lc")) == {
        "KRdiag", "Kxidiag", "LambdaDiag", "kd", "ks", "alpha"
    }
    with pytest.raises(ValueError):
        optimized_gains("other", "lc")


def test_nominal_scenario_validation_rejects_unknown_mode():
    scenario = default_scenario(duration=0.02)
    scenario["controller"]["mode"] = "other"
    with pytest.raises(ValueError):
        validate_scenario(scenario)
