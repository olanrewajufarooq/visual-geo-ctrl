"""Vectorized, parallel Particle Swarm Optimizer matching MATLAB particleswarm."""

from typing import Callable, Optional, Tuple, List
from concurrent.futures import ProcessPoolExecutor
import numpy as np


class ParticleSwarmOptimizer:
    """Clerc-Kennedy constriction Particle Swarm Optimizer with seed injection and parallel evaluation."""

    def __init__(
        self,
        cost_func: Callable[[np.ndarray], float],
        lower_bound: np.ndarray,
        upper_bound: np.ndarray,
        swarm_size: int = 50,
        max_iter: int = 20,
        max_stall: int = 10,
        tol: float = 1e-3,
        initial_points: Optional[np.ndarray] = None,
        parallel: bool = True,
        max_workers: Optional[int] = None,
        verbose: bool = True,
    ):
        self.cost_func = cost_func
        self.lb = np.asarray(lower_bound, dtype=float).ravel()
        self.ub = np.asarray(upper_bound, dtype=float).ravel()
        if len(self.lb) != len(self.ub) or np.any(self.lb >= self.ub):
            raise ValueError("Bounds must have equal lengths with lower_bound < upper_bound.")

        self.dim = len(self.lb)
        self.swarm_size = max(5, int(swarm_size))
        self.max_iter = max(1, int(max_iter))
        self.max_stall = max(1, int(max_stall))
        self.tol = float(tol)
        self.initial_points = initial_points
        self.parallel = bool(parallel)
        self.max_workers = max_workers
        self.verbose = bool(verbose)

        # Standard Clerc & Kennedy constriction coefficients matching MATLAB
        self.w = 0.72984
        self.c1 = 1.49618
        self.c2 = 1.49618

    def optimize(self) -> Tuple[np.ndarray, float, List[float]]:
        """Run PSO search, returning (best_candidate, best_cost, cost_history)."""
        span = self.ub - self.lb

        # 1. Initialize particle positions
        X = np.random.uniform(self.lb, self.ub, size=(self.swarm_size, self.dim))

        # Inject seeds if provided
        if self.initial_points is not None:
            pts = np.atleast_2d(self.initial_points)
            pts = np.clip(pts, self.lb, self.ub)
            n_seeds = min(len(pts), self.swarm_size)
            X[:n_seeds] = pts[:n_seeds]

        # 2. Initialize particle velocities (bounded to 20% of range)
        V = np.random.uniform(-0.2 * span, 0.2 * span, size=(self.swarm_size, self.dim))

        # 3. Evaluate initial population
        costs = self._evaluate_batch(X)
        pbest_X = np.copy(X)
        pbest_cost = np.copy(costs)

        gbest_idx = int(np.argmin(pbest_cost))
        gbest_X = np.copy(pbest_X[gbest_idx])
        gbest_cost = float(pbest_cost[gbest_idx])

        stall_count = 0
        history = [gbest_cost]

        if self.verbose:
            print(f"      PSO Init: Best Cost = {gbest_cost:.6g}")

        # 4. Swarm iteration loop
        for it in range(1, self.max_iter + 1):
            r1 = np.random.uniform(0.0, 1.0, size=(self.swarm_size, self.dim))
            r2 = np.random.uniform(0.0, 1.0, size=(self.swarm_size, self.dim))

            # Velocity update
            V = self.w * V + self.c1 * r1 * (pbest_X - X) + self.c2 * r2 * (gbest_X - X)
            # Velocity clamping to range
            max_v = 0.5 * span
            V = np.clip(V, -max_v, max_v)

            # Position update
            X_new = X + V

            # Bound constraints and velocity reflection/damping
            hit_lower = X_new < self.lb
            hit_upper = X_new > self.ub
            X = np.clip(X_new, self.lb, self.ub)
            V[hit_lower | hit_upper] *= -0.5

            costs = self._evaluate_batch(X)

            # Update personal bests
            improved_pbest = costs < pbest_cost
            pbest_X[improved_pbest] = X[improved_pbest]
            pbest_cost[improved_pbest] = costs[improved_pbest]

            # Update global best
            cur_best_idx = int(np.argmin(pbest_cost))
            cur_best_cost = float(pbest_cost[cur_best_idx])

            if gbest_cost - cur_best_cost > self.tol:
                stall_count = 0
                gbest_cost = cur_best_cost
                gbest_X = np.copy(pbest_X[cur_best_idx])
            else:
                stall_count += 1

            history.append(gbest_cost)
            if self.verbose:
                print(f"      PSO Iter {it:2d}/{self.max_iter}: Best = {gbest_cost:.6g} (stall {stall_count}/{self.max_stall})")

            if stall_count >= self.max_stall:
                if self.verbose:
                    print(f"      PSO Converged: Stalled for {self.max_stall} iterations.")
                break

        return gbest_X, gbest_cost, history

    def _evaluate_batch(self, X: np.ndarray) -> np.ndarray:
        """Evaluate a batch of candidate positions."""
        rows = [row for row in X]
        if self.parallel and len(rows) > 1:
            with ProcessPoolExecutor(max_workers=self.max_workers) as executor:
                costs = list(executor.map(self.cost_func, rows))
        else:
            costs = [self.cost_func(r) for r in rows]
        return np.array(costs, dtype=float)
