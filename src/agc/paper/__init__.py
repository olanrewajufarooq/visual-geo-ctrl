"""Paper equations, error kinematics, regressors, and adaptive controllers."""

from .coriolis import coriolis
from .errors import potential, potential_derivative
from .regressor import regressor, inertial_block, coriolis_block, gravity_block
from .adaptation import euclidean_step, bregman_step, pseudo_gradient
from .controller import controller, parameter_state, ControllerDiagnostics

__all__ = [
    "coriolis",
    "potential",
    "potential_derivative",
    "regressor",
    "inertial_block",
    "coriolis_block",
    "gravity_block",
    "euclidean_step",
    "bregman_step",
    "pseudo_gradient",
    "controller",
    "parameter_state",
    "ControllerDiagnostics",
]
