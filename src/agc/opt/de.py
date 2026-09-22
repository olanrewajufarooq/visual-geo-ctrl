"""SciPy Differential Evolution and local simplex polishing engine."""

from typing import Callable, Optional, Tuple, List
import numpy as np
from scipy.optimize import differential_evolution, minimize


class DifferentialEvolutionOptimizer:
    """SciPy Differential Evolution wrapper supporting seed injection and multi-core workers."""

    def __init__(
        self,
        cost_func: Callable[[np.ndarray], float],
        lower_bound: np.ndarray,
        upper_bound: np.ndarray,
        pop_size: int = 15,
        max_iter: int = 20,
        tol: float = 1e-3,
        initial_points: Optional[np.ndarray] = None,
        parallel: bool = True,
        max_workers: Optional[int] = None,
        verbose: bool = True,
        seed: Optional[int] = None,
    ):
        self.cost_func = cost_func
        self.lb = np.asarray(lower_bound, dtype=float).ravel()
        self.ub = np.asarray(upper_bound, dtype=float).ravel()
        self.dim = len(self.lb)
        self.pop_size = pop_size
        self.max_iter = max_iter
        self.tol = tol
        self.initial_points = initial_points
        self.parallel = parallel
        self.max_workers = max_workers
        self.verbose = verbose
        self.seed = seed

    def optimize(self) -> Tuple[np.ndarray, float, List[float]]:
        bounds = list(zip(self.lb, self.ub))
        workers = -1 if self.parallel else 1
        history = []
        rng = np.random.default_rng(self.seed)

        init_method = "latinhypercube"
        if self.initial_points is not None:
            pts = np.atleast_2d(self.initial_points)
            pts = np.clip(pts, self.lb, self.ub)
            total_needed = self.pop_size * self.dim
            if len(pts) < total_needed:
                extra = rng.uniform(self.lb, self.ub, size=(total_needed - len(pts), self.dim))
                init_pop = np.vstack([pts, extra])
            else:
                init_pop = pts[:total_needed]
            init_method = init_pop

        def _callback(xk, convergence=None):
            c = self.cost_func(xk)
            history.append(float(c))
            if self.verbose:
                print(f"      DE Iter: Best Cost = {c:.6g}")

        res = differential_evolution(
            self.cost_func,
            bounds=bounds,
            maxiter=self.max_iter,
            popsize=self.pop_size,
            tol=self.tol,
            init=init_method,
            workers=workers,
            updating="deferred",
            callback=_callback,
            polish=False,
            seed=self.seed,
        )

        best_x = np.asarray(res.x, dtype=float)
        best_cost = float(res.fun)
        return best_x, best_cost, history


def nelder_mead_polish(
    cost_func: Callable[[np.ndarray], float],
    initial_point: np.ndarray,
    lower_bound: np.ndarray,
    upper_bound: np.ndarray,
    max_iter: int = 50,
    tol: float = 1e-4,
    verbose: bool = True,
) -> Tuple[np.ndarray, float]:
    """Perform local Nelder-Mead simplex search starting from the best incumbent."""
    lb = np.asarray(lower_bound, dtype=float).ravel()
    ub = np.asarray(upper_bound, dtype=float).ravel()

    def _bounded_cost(x):
        if np.any(x < lb) or np.any(x > ub):
            return 1e6
        return cost_func(x)

    x0 = np.clip(np.asarray(initial_point, dtype=float).ravel(), lb, ub)
    init_cost = cost_func(x0)

    if verbose:
        print(f"      Nelder-Mead Polish starting at cost = {init_cost:.6g}")

    res = minimize(
        _bounded_cost,
        x0=x0,
        method="Nelder-Mead",
        options={"maxiter": max_iter, "xatol": tol, "fatol": tol, "disp": False},
    )

    polished_x = np.clip(np.asarray(res.x, dtype=float), lb, ub)
    polished_cost = float(cost_func(polished_x))
    if polished_cost < init_cost:
        if verbose:
            print(f"      Polish Improved: {init_cost:.6g} -> {polished_cost:.6g}")
        return polished_x, polished_cost
    return x0, init_cost
