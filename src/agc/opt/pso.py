"""Vectorized, parallel Particle Swarm Optimizer matching MATLAB particleswarm."""

from copy import deepcopy
from typing import Callable, Optional, Tuple, List, Dict, Any
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
        seed: Optional[int] = None,
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
        self.seed = seed
        self.rng = np.random.default_rng(seed)
        self._executor = None
        self._cost_cache = {}

        # Standard Clerc & Kennedy constriction coefficients matching MATLAB
        self.w = 0.72984
        self.c1 = 1.49618
        self.c2 = 1.49618

    def optimize(
        self,
        resume_state: Optional[Dict[str, Any]] = None,
        checkpoint_callback: Optional[Callable[[Dict[str, Any]], None]] = None,
    ) -> Tuple[np.ndarray, float, List[float]]:
        """Run PSO, optionally restoring a checkpoint and reporting safe checkpoints."""
        if self.parallel:
            self._executor = ProcessPoolExecutor(max_workers=self.max_workers)
        try:
            return self._optimize(resume_state, checkpoint_callback)
        finally:
            self.close()

    def _optimize(
        self,
        resume_state: Optional[Dict[str, Any]] = None,
        checkpoint_callback: Optional[Callable[[Dict[str, Any]], None]] = None,
    ) -> Tuple[np.ndarray, float, List[float]]:
        """Run the PSO loop with one worker pool shared by all evaluations."""
        span = self.ub - self.lb

        if resume_state is not None:
            self._validate_resume_state(resume_state)
            self.rng.bit_generator.state = deepcopy(resume_state["rng_state"])
            X = np.asarray(resume_state["positions"], dtype=float)
            V = np.asarray(resume_state["velocities"], dtype=float)
            pbest_X = np.asarray(resume_state["personal_best_positions"], dtype=float)
            pbest_cost = np.asarray(resume_state["personal_best_costs"], dtype=float)
            gbest_X = np.asarray(resume_state["global_best_position"], dtype=float)
            gbest_cost = float(resume_state["global_best_cost"])
            stall_count = int(resume_state["stall_count"])
            history = [float(x) for x in resume_state["history"]]
            start_iteration = int(resume_state["iteration"]) + 1
            if bool(resume_state.get("complete", False)):
                return gbest_X, gbest_cost, history
        else:
            # 1. Initialize particle positions.
            X = self.rng.uniform(self.lb, self.ub, size=(self.swarm_size, self.dim))

            # Inject seeds if provided.
            if self.initial_points is not None:
                pts = np.atleast_2d(self.initial_points)
                pts = np.clip(pts, self.lb, self.ub)
                n_seeds = min(len(pts), self.swarm_size)
                X[:n_seeds] = pts[:n_seeds]

            # 2. Initialize particle velocities (bounded to 20% of range).
            V = self.rng.uniform(-0.2 * span, 0.2 * span, size=(self.swarm_size, self.dim))

            # 3. Evaluate initial population.
            costs = self._evaluate_batch(X)
            pbest_X = np.copy(X)
            pbest_cost = np.copy(costs)

            gbest_idx = int(np.argmin(pbest_cost))
            gbest_X = np.copy(pbest_X[gbest_idx])
            gbest_cost = float(pbest_cost[gbest_idx])

            stall_count = 0
            history = [gbest_cost]
            start_iteration = 1
            self._report_checkpoint(
                checkpoint_callback, 0, X, V, pbest_X, pbest_cost, gbest_X,
                gbest_cost, stall_count, history, complete=False,
            )

            if self.verbose:
                print(f"      PSO Init: Best Cost = {gbest_cost:.6g}")

        # 4. Swarm iteration loop
        for it in range(start_iteration, self.max_iter + 1):
            r1 = self.rng.uniform(0.0, 1.0, size=(self.swarm_size, self.dim))
            r2 = self.rng.uniform(0.0, 1.0, size=(self.swarm_size, self.dim))

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
            complete = stall_count >= self.max_stall or it >= self.max_iter
            self._report_checkpoint(
                checkpoint_callback, it, X, V, pbest_X, pbest_cost, gbest_X,
                gbest_cost, stall_count, history, complete=complete,
            )
            if self.verbose:
                print(f"      PSO Iter {it:2d}/{self.max_iter}: Best = {gbest_cost:.6g} (stall {stall_count}/{self.max_stall})")

            if stall_count >= self.max_stall:
                if self.verbose:
                    print(f"      PSO Converged: Stalled for {self.max_stall} iterations.")
                break

        return gbest_X, gbest_cost, history

    def _validate_resume_state(self, state: Dict[str, Any]) -> None:
        """Reject swarm checkpoints created for a different PSO configuration."""
        if state.get("schemaVersion") != 1:
            raise ValueError("Unsupported PSO checkpoint schema.")
        if not np.array_equal(np.asarray(state.get("lower_bound"), dtype=float), self.lb):
            raise ValueError("PSO checkpoint lower bounds do not match this run.")
        if not np.array_equal(np.asarray(state.get("upper_bound"), dtype=float), self.ub):
            raise ValueError("PSO checkpoint upper bounds do not match this run.")
        if int(state.get("swarm_size", -1)) != self.swarm_size:
            raise ValueError("PSO checkpoint swarm size does not match this run.")
        if int(state.get("max_iter", -1)) != self.max_iter:
            raise ValueError("PSO checkpoint iteration limit does not match this run.")
        expected_shapes = {
            "positions": (self.swarm_size, self.dim),
            "velocities": (self.swarm_size, self.dim),
            "personal_best_positions": (self.swarm_size, self.dim),
            "personal_best_costs": (self.swarm_size,),
            "global_best_position": (self.dim,),
        }
        for key, shape in expected_shapes.items():
            if np.asarray(state.get(key), dtype=float).shape != shape:
                raise ValueError(f"PSO checkpoint field {key} has an invalid shape.")

    def _report_checkpoint(
        self,
        callback: Optional[Callable[[Dict[str, Any]], None]],
        iteration: int,
        positions: np.ndarray,
        velocities: np.ndarray,
        personal_best_positions: np.ndarray,
        personal_best_costs: np.ndarray,
        global_best_position: np.ndarray,
        global_best_cost: float,
        stall_count: int,
        history: List[float],
        complete: bool,
    ) -> None:
        if callback is None:
            return
        callback({
            "schemaVersion": 1,
            "lower_bound": self.lb.copy(),
            "upper_bound": self.ub.copy(),
            "swarm_size": self.swarm_size,
            "max_iter": self.max_iter,
            "iteration": int(iteration),
            "positions": positions.copy(),
            "velocities": velocities.copy(),
            "personal_best_positions": personal_best_positions.copy(),
            "personal_best_costs": personal_best_costs.copy(),
            "global_best_position": global_best_position.copy(),
            "global_best_cost": float(global_best_cost),
            "stall_count": int(stall_count),
            "history": list(history),
            "rng_state": deepcopy(self.rng.bit_generator.state),
            "complete": bool(complete),
        })

    def _evaluate_batch(self, X: np.ndarray) -> np.ndarray:
        """Evaluate a batch of candidate positions."""
        rows = [row for row in X]
        keys = [tuple(np.asarray(row, dtype=float).tolist()) for row in rows]
        missing = []
        missing_keys = []
        for row, key in zip(rows, keys):
            if key not in self._cost_cache and key not in missing_keys:
                missing.append(row)
                missing_keys.append(key)

        if missing:
            if self.parallel and self._executor is not None and len(missing) > 1:
                new_costs = list(self._executor.map(self.cost_func, missing))
            else:
                new_costs = [self.cost_func(row) for row in missing]
            self._cost_cache.update(zip(missing_keys, (float(c) for c in new_costs)))

        return np.array([self._cost_cache[key] for key in keys], dtype=float)

    def close(self):
        """Release the persistent worker pool after one optimization run."""
        if self._executor is not None:
            self._executor.shutdown(wait=True)
            self._executor = None

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass
