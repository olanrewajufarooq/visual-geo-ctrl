"""Gain optimization package for adaptive geometric tracking control."""

from .bounds import (
    gain_bounds,
    gain_block_indices,
    gain_optimization_stages,
    expand_scenario_selection,
)
from .encoding import (
    encode_scenario_gains,
    apply_scenario_gains,
    apply_gain_block,
    round_gains,
    round_significant,
)
from .objective import (
    objective_scales,
    objective_weights,
    optimization_options,
    evaluate_scenario_candidate,
    best_feasible_candidate,
)
from .pso import ParticleSwarmOptimizer
from .bregman_profile import profile_bregman_gain, bregman_gamma_grid
from .staged_optimizer import run_staged_optimization, promote_gains_to_registry

__all__ = [
    "gain_bounds",
    "gain_block_indices",
    "gain_optimization_stages",
    "expand_scenario_selection",
    "encode_scenario_gains",
    "apply_scenario_gains",
    "apply_gain_block",
    "round_gains",
    "round_significant",
    "objective_scales",
    "objective_weights",
    "optimization_options",
    "evaluate_scenario_candidate",
    "best_feasible_candidate",
    "ParticleSwarmOptimizer",
    "profile_bregman_gain",
    "bregman_gamma_grid",
    "run_staged_optimization",
    "promote_gains_to_registry",
]
