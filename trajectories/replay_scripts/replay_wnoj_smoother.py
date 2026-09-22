"""Sparse nonlinear SE(3) White-Noise-On-Jerk (WNOJ) batch smoother.

Full Python implementation of Tang, Yoon, and Barfoot (IEEE RA-L 2019):
"A White-Noise-on-Jerk Motion Prior for Continuous-Time Trajectory Estimation on SE(3)".

Convention:
    The public API uses the project convention:
        Hdot = H * hat(V), where H maps body coordinates to world coordinates,
        V = [omega_b; v_b] in R^6,
        A = Vdot = [alpha_b; a_b] in R^6.
    Internally, the smoother uses the paper convention (Tang et al.):
        T = inv(H),  varpi = -V,  dotvarpi = -A,
        Tdot = hat(varpi) * T.
"""

from typing import Dict, Any, Optional, Tuple, List, Union
import warnings
import numpy as np
import scipy.linalg as la
import scipy.sparse as sp
import scipy.sparse.linalg as spla

from agc.math.se3 import (
    skew,
    unskew,
    ad_twist,
    inv_se3,
    expm_so3,
    rotm_to_quat,
    quat_to_rotm,
    log_so3,
    exp_se3,
    log_se3,
)


class ReplayWnojSmoother:
    """Sparse nonlinear SE(3) WNOJ batch smoother and continuous GP interpolator."""

    def __init__(self, options: Optional[Dict[str, Any]] = None):
        if options is None:
            options = {}
        self.options = self._resolve_options(options)
        self.method = "wnoj"

        # Fitted trajectory state
        self.t: Optional[np.ndarray] = None
        self.output_times: Optional[np.ndarray] = None
        self.H: Optional[np.ndarray] = None
        self.V: Optional[np.ndarray] = None
        self.A: Optional[np.ndarray] = None
        self.diagnostics: Dict[str, Any] = {}

        # Internal paper states (K knots)
        self._paper_T: Optional[np.ndarray] = None          # (4, 4, K)
        self._paper_W: Optional[np.ndarray] = None          # (6, K)
        self._paper_D: Optional[np.ndarray] = None          # (6, K)

    @property
    def outputTimes(self) -> Optional[np.ndarray]:
        """MATLAB backward-compatibility alias."""
        return self.output_times

    def _resolve_options(self, supplied: Dict[str, Any]) -> Dict[str, Any]:
        """Resolve and validate smoother options with defaults matching MATLAB."""
        defaults = {
            "knotIntervalSeconds": 0.01,
            "sigmaPositionMeters": 0.01,
            "sigmaOrientationRadians": 0.01,
            "sigmaLinearVelocityMps": 0.10,
            "sigmaAngularVelocityRadps": 0.10,
            "jerkSpectralDensityAngular": 1.0,
            "jerkSpectralDensityLinear": 1.0,
            "maxIterations": 200,
            "maxDampingTrials": 8,
            "stepTolerance": 1e-5,
            "gradientTolerance": 1e-4,
            "relativeCostTolerance": 1e-6,
            "initialDamping": 1e-6,
            "minimumDamping": 1e-12,
            "finiteDifferenceStep": 1e-6,
            "requireConvergence": False,
            "warnOnNonConvergence": True,
        }

        # Map snake_case alternatives to canonical camelCase names
        aliases = {
            "knot_interval_seconds": "knotIntervalSeconds",
            "sigma_position_meters": "sigmaPositionMeters",
            "sigma_orientation_radians": "sigmaOrientationRadians",
            "sigma_linear_velocity_mps": "sigmaLinearVelocityMps",
            "sigma_angular_velocity_radps": "sigmaAngularVelocityRadps",
            "jerk_spectral_density_angular": "jerkSpectralDensityAngular",
            "jerk_spectral_density_linear": "jerkSpectralDensityLinear",
            "max_iterations": "maxIterations",
            "max_damping_trials": "maxDampingTrials",
            "step_tolerance": "stepTolerance",
            "gradient_tolerance": "gradientTolerance",
            "relative_cost_tolerance": "relativeCostTolerance",
            "initial_damping": "initialDamping",
            "minimum_damping": "minimumDamping",
            "finite_difference_step": "finiteDifferenceStep",
            "require_convergence": "requireConvergence",
            "warn_on_non_convergence": "warnOnNonConvergence",
        }

        opts = dict(defaults)
        for k, v in supplied.items():
            canonical_k = aliases.get(k, k)
            opts[canonical_k] = v

        if "poseSigma" in supplied:
            opts["sigmaPositionMeters"] = supplied["poseSigma"]
            opts["sigmaOrientationRadians"] = supplied["poseSigma"]
        if "twistSigma" in supplied:
            opts["sigmaLinearVelocityMps"] = supplied["twistSigma"]
            opts["sigmaAngularVelocityRadps"] = supplied["twistSigma"]

        # Jerk spectral density Qc (6x6 PSD)
        if "jerkSpectralDensity" in supplied and supplied["jerkSpectralDensity"] is not None:
            Qc = supplied["jerkSpectralDensity"]
            if np.isscalar(Qc):
                Qc = float(Qc) * np.eye(6)
            else:
                Qc = np.asarray(Qc, dtype=float)
        else:
            ang = opts["jerkSpectralDensityAngular"]
            ang_vec = np.full(3, float(ang)) if np.isscalar(ang) else np.asarray(ang, dtype=float).ravel()
            lin = opts["jerkSpectralDensityLinear"]
            lin_vec = np.full(3, float(lin)) if np.isscalar(lin) else np.asarray(lin, dtype=float).ravel()
            Qc = np.diag(np.concatenate([ang_vec, lin_vec]))

        opts["jerkSpectralDensity"] = Qc
        opts["poseStd"] = np.concatenate([
            np.full(3, float(opts["sigmaOrientationRadians"])),
            np.full(3, float(opts["sigmaPositionMeters"])),
        ])
        opts["twistStd"] = np.concatenate([
            np.full(3, float(opts["sigmaAngularVelocityRadps"])),
            np.full(3, float(opts["sigmaLinearVelocityMps"])),
        ])
        return opts

    def fit(
        self,
        t: np.ndarray,
        R: np.ndarray,
        p: np.ndarray,
        V_body: np.ndarray,
    ) -> "ReplayWnojSmoother":
        """Fit continuous SE(3) WNOJ trajectory to discrete measurements.

        Parameters
        ----------
        t : (N,) array_like
            Strictly increasing measurement timestamps.
        R : (3, 3, N) or (N, 3, 3) array_like
            Body-to-world rotation matrices.
        p : (N, 3) array_like
            World positions.
        V_body : (N, 6) array_like
            Body-frame twists [omega_b; v_b].
        """
        meas_times, H_all, V_body_arr = self._validate_measurements(t, R, p, V_body)
        knot_indices, knot_times = self._select_knots(meas_times, self.options["knotIntervalSeconds"])
        K = len(knot_times)

        H_meas = H_all[:, :, knot_indices]
        V_project_meas = V_body_arr[knot_indices, :].T  # (6, K)
        T_meas, W_meas = self._to_paper_measurements(H_meas, V_project_meas)
        T, W, D = self._initialize_states(knot_times, T_meas, W_meas)

        current_cost = self._batch_cost(T, W, D, knot_times, T_meas, W_meas)
        cost_history = [current_cost]
        step_history = []
        damping_history = []
        converged = False
        termination_reason = "maximum iterations"
        normal_nnz = 0
        state_dim = 18 * K
        damping = float(self.options["initialDamping"])
        grad_inf_norm = float("nan")

        max_iter = int(self.options["maxIterations"])
        max_damping_trials = int(self.options["maxDampingTrials"])
        grad_tol = float(self.options["gradientTolerance"])
        step_tol = float(self.options["stepTolerance"])
        cost_tol = float(self.options["relativeCostTolerance"])
        min_damping = float(self.options["minimumDamping"])

        for iteration in range(max_iter):
            residual, jacobian = self._linearize_batch(T, W, D, knot_times, T_meas, W_meas)
            normal_matrix = jacobian.T @ jacobian
            gradient = jacobian.T @ residual
            grad_inf_norm = float(np.max(np.abs(gradient)))
            normal_nnz = normal_matrix.nnz

            if grad_inf_norm <= grad_tol:
                converged = True
                termination_reason = "gradient tolerance"
                break

            diag_normal = normal_matrix.diagonal()
            diagonal_scale = np.maximum(diag_normal, 1.0)
            accepted = False
            accepted_step = None
            accepted_cost = current_cost
            accepted_damping = damping

            for trial in range(max_damping_trials):
                damping_matrix = sp.diags(damping * diagonal_scale, 0, shape=(state_dim, state_dim), format="csr")
                A_damped = normal_matrix + damping_matrix
                try:
                    delta = spla.spsolve(A_damped, -gradient)
                except Exception:
                    damping *= 10.0
                    continue

                if not np.all(np.isfinite(delta)):
                    damping *= 10.0
                    continue

                T_trial, W_trial, D_trial = self._retract(T, W, D, delta)
                trial_cost = self._batch_cost(T_trial, W_trial, D_trial, knot_times, T_meas, W_meas)
                if np.isfinite(trial_cost) and trial_cost < current_cost:
                    accepted = True
                    accepted_step = delta
                    accepted_cost = trial_cost
                    accepted_damping = damping
                    T = T_trial
                    W = W_trial
                    D = D_trial
                    break
                damping *= 10.0

            if not accepted:
                termination_reason = "no decreasing LM step"
                break

            prev_cost = current_cost
            current_cost = accepted_cost
            cost_history.append(current_cost)
            step_norm = float(np.max(np.abs(accepted_step)))
            step_history.append(step_norm)
            damping_history.append(accepted_damping)
            damping = max(min_damping, accepted_damping / 3.0)

            if step_norm <= step_tol:
                converged = True
                termination_reason = "step tolerance"
                break

            rel_decrease = (prev_cost - current_cost) / max(1.0, prev_cost)
            if rel_decrease <= cost_tol:
                converged = True
                termination_reason = "cost tolerance"
                break

        if not converged and self.options["requireConvergence"]:
            raise RuntimeError(f"WNOJ optimization stopped without convergence: {termination_reason}.")

        self.t = knot_times
        self.output_times = meas_times
        self._paper_T = T
        self._paper_W = W
        self._paper_D = D
        self.H, self.V, self.A = self._paper_arrays_to_project(T, W, D)

        final_step = step_history[-1] if step_history else float("nan")
        final_rel_dec = self._final_relative_decrease(cost_history)

        self.diagnostics = {
            "converged": converged,
            "terminationReason": termination_reason,
            "iterations": len(step_history),
            "initialCost": cost_history[0],
            "finalCost": current_cost,
            "costHistory": cost_history,
            "stepHistory": step_history,
            "dampingHistory": damping_history,
            "finalStepInfinityNorm": final_step,
            "finalRelativeCostDecrease": final_rel_dec,
            "linearizationGradientInfinityNorm": grad_inf_norm,
            "knotCount": K,
            "stateDimension": state_dim,
            "normalNnz": normal_nnz,
        }

        if not converged and self.options["warnOnNonConvergence"]:
            warnings.warn(
                f"WNOJ optimization stopped after {len(step_history)} iterations ({termination_reason}). "
                f"Final relative cost decrease {final_rel_dec:.3g}; final step {final_step:.3g}."
            )

        return self

    def evaluate(self, tq: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Evaluate smooth H (4x4), V (6,), A (6,) at query time tq.

        Parameters
        ----------
        tq : float
            Evaluation time in seconds.

        Returns
        -------
        H : (4, 4) ndarray
            Pose mapping body coordinates to world coordinates.
        V : (6,) ndarray
            Left-trivialized body twist [omega_b; v_b].
        A : (6,) ndarray
            Body acceleration [alpha_b; a_b].
        """
        if self.t is None or self._paper_T is None:
            raise RuntimeError("The smoother must be fitted before evaluation.")

        tq = float(np.clip(tq, self.t[0], self.t[-1]))
        if tq <= self.t[0]:
            return self._paper_state_to_project(
                self._paper_T[:, :, 0], self._paper_W[:, 0], self._paper_D[:, 0]
            )
        if tq >= self.t[-1]:
            last = len(self.t) - 1
            return self._paper_state_to_project(
                self._paper_T[:, :, last], self._paper_W[:, last], self._paper_D[:, last]
            )

        i = int(np.searchsorted(self.t, tq, side="right") - 1)
        i = max(0, min(i, len(self.t) - 2))
        dt = self.t[i + 1] - self.t[i]
        tau = tq - self.t[i]

        T, W, D = self._interpolate(
            self._paper_T[:, :, i],
            self._paper_W[:, i],
            self._paper_D[:, i],
            self._paper_T[:, :, i + 1],
            self._paper_W[:, i + 1],
            self._paper_D[:, i + 1],
            dt,
            tau,
        )
        return self._paper_state_to_project(T, W, D)

    def transition(self, dt: float, Qc: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Evaluate state transition matrix Phi(dt) and covariance Q(dt)."""
        I6 = np.eye(6, dtype=float)
        Phi = np.block([
            [I6, dt * I6, 0.5 * (dt**2) * I6],
            [np.zeros((6, 6)), I6, dt * I6],
            [np.zeros((6, 6)), np.zeros((6, 6)), I6],
        ])
        Q = np.block([
            [(dt**5 / 20.0) * Qc, (dt**4 / 8.0) * Qc, (dt**3 / 6.0) * Qc],
            [(dt**4 / 8.0) * Qc,  (dt**3 / 3.0) * Qc, (dt**2 / 2.0) * Qc],
            [(dt**3 / 6.0) * Qc,  (dt**2 / 2.0) * Qc, dt * Qc],
        ])
        return Phi, Q

    def prior_residual(
        self,
        Hi: np.ndarray,
        Vi: np.ndarray,
        Ai: np.ndarray,
        Hj: np.ndarray,
        Vj: np.ndarray,
        Aj: np.ndarray,
        dt: float,
    ) -> np.ndarray:
        """Evaluate raw prior residual between two project states."""
        Ti, Wi, Di = self._project_state_to_paper(Hi, Vi, Ai)
        Tj, Wj, Dj = self._project_state_to_paper(Hj, Vj, Aj)
        return self._paper_prior_residual(Ti, Wi, Di, Tj, Wj, Dj, dt)

    def priorResidual(self, *args, **kwargs) -> np.ndarray:
        """MATLAB backward-compatibility alias."""
        return self.prior_residual(*args, **kwargs)

    def validate(self) -> Dict[str, float]:
        """Validate internal kinematic consistency on output timestamps."""
        if self.output_times is None:
            raise RuntimeError("The smoother must be fitted before validation.")

        times = self.output_times
        N = len(times)
        if N < 2:
            return {"maxPoseTwistResidual": 0.0, "maxTwistAccelerationResidual": 0.0}

        max_pose_twist = 0.0
        max_twist_accel = 0.0

        for k in range(N - 1):
            dt = times[k + 1] - times[k]
            H0, V0, A0 = self.evaluate(times[k])
            H1, V1, _ = self.evaluate(times[k + 1])

            # Pose-twist consistency: log(H0^{-1} * H1) / dt - V0
            step_H = inv_se3(H0) @ H1
            res_pv = np.max(np.abs(log_se3(step_H) / dt - V0))
            if res_pv > max_pose_twist:
                max_pose_twist = float(res_pv)

            # Twist-acceleration consistency: (V1 - V0) / dt - A0
            res_va = np.max(np.abs((V1 - V0) / dt - A0))
            if res_va > max_twist_accel:
                max_twist_accel = float(res_va)

        return {
            "maxPoseTwistResidual": max_pose_twist,
            "maxTwistAccelerationResidual": max_twist_accel,
        }

    # ── Internal Core Implementation ──────────────────────────────────────────

    def _validate_measurements(
        self,
        t: np.ndarray,
        R: np.ndarray,
        p: np.ndarray,
        V: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        t = np.asarray(t, dtype=float).ravel()
        n = len(t)
        p = np.asarray(p, dtype=float)
        V = np.asarray(V, dtype=float)

        if n < 3:
            raise ValueError(f"At least 3 measurement samples required, got {n}.")
        if np.any(np.diff(t) <= 0.0):
            raise ValueError("Measurement timestamps must be strictly increasing.")
        if p.shape != (n, 3) or V.shape != (n, 6):
            raise ValueError(f"Shape mismatch: t={n}, p={p.shape}, V={V.shape}.")

        if R.ndim == 3 and R.shape == (3, 3, n):
            R_arr = R
        elif R.ndim == 3 and R.shape == (n, 3, 3):
            R_arr = np.transpose(R, (1, 2, 0))
        else:
            raise ValueError(f"Incompatible rotation shape: {R.shape} for {n} samples.")

        H = np.zeros((4, 4, n), dtype=float)
        for k in range(n):
            H[0:3, 0:3, k] = R_arr[:, :, k]
            H[0:3, 3, k] = p[k]
            H[3, 3, k] = 1.0

        return t, H, V

    def _select_knots(self, t: np.ndarray, interval: float) -> Tuple[np.ndarray, np.ndarray]:
        indices = [0]
        last_t = t[0]
        n = len(t)
        for k in range(1, n - 1):
            if t[k] - last_t >= interval:
                indices.append(k)
                last_t = t[k]
        if indices[-1] != n - 1:
            indices.append(n - 1)

        idx_arr = np.array(indices, dtype=int)
        if len(idx_arr) < 3:
            idx_arr = np.unique(np.round(np.linspace(0, n - 1, 3)).astype(int))
        return idx_arr, t[idx_arr]

    def _to_paper_measurements(
        self,
        H: np.ndarray,
        V_project: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        K = H.shape[2]
        T = np.zeros((4, 4, K), dtype=float)
        for k in range(K):
            T[:, :, k] = inv_se3(H[:, :, k])
        W = -np.asarray(V_project, dtype=float)
        return T, W

    def _initialize_states(
        self,
        t: np.ndarray,
        T_meas: np.ndarray,
        W_meas: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        K = len(t)
        T = np.copy(T_meas)
        W = np.copy(W_meas)
        D = np.zeros((6, K), dtype=float)
        for k in range(1, K - 1):
            D[:, k] = (W[:, k + 1] - W[:, k - 1]) / (t[k + 1] - t[k - 1])
        D[:, 0] = (W[:, 1] - W[:, 0]) / (t[1] - t[0])
        D[:, K - 1] = (W[:, K - 1] - W[:, K - 2]) / (t[K - 1] - t[K - 2])
        return T, W, D

    def _batch_cost(
        self,
        T: np.ndarray,
        W: np.ndarray,
        D: np.ndarray,
        t: np.ndarray,
        T_meas: np.ndarray,
        W_meas: np.ndarray,
    ) -> float:
        res = self._residual_vector(T, W, D, t, T_meas, W_meas)
        return float(0.5 * np.dot(res, res))

    def _residual_vector(
        self,
        T: np.ndarray,
        W: np.ndarray,
        D: np.ndarray,
        t: np.ndarray,
        T_meas: np.ndarray,
        W_meas: np.ndarray,
    ) -> np.ndarray:
        K = len(t)
        res_list = []
        for k in range(K):
            res_list.append(self._measurement_residual(T[:, :, k], W[:, k], T_meas[:, :, k], W_meas[:, k]))
        for k in range(K - 1):
            dt = t[k + 1] - t[k]
            res_list.append(self._prior_whitened_residual(
                T[:, :, k], W[:, k], D[:, k], T[:, :, k + 1], W[:, k + 1], D[:, k + 1], dt
            ))
        return np.concatenate(res_list)

    def _linearize_batch(
        self,
        T: np.ndarray,
        W: np.ndarray,
        D: np.ndarray,
        t: np.ndarray,
        T_meas: np.ndarray,
        W_meas: np.ndarray,
    ) -> Tuple[np.ndarray, sp.csr_matrix]:
        K = len(t)
        row_count = 12 * K + 18 * (K - 1)
        col_count = 18 * K

        res = np.zeros(row_count, dtype=float)
        jac_data = []
        jac_rows = []
        jac_cols = []

        curr_row = 0

        # Measurement factors
        for k in range(K):
            f_res, f_jac = self._linearize_measurement(T[:, :, k], W[:, k], T_meas[:, :, k], W_meas[:, k])
            res[curr_row : curr_row + 12] = f_res

            c_offset = 18 * k
            for r in range(12):
                for c in range(18):
                    val = f_jac[r, c]
                    if val != 0.0:
                        jac_data.append(val)
                        jac_rows.append(curr_row + r)
                        jac_cols.append(c_offset + c)
            curr_row += 12

        # Prior factors
        for k in range(K - 1):
            dt = t[k + 1] - t[k]
            f_res, f_jac = self._linearize_prior(
                T[:, :, k], W[:, k], D[:, k], T[:, :, k + 1], W[:, k + 1], D[:, k + 1], dt
            )
            res[curr_row : curr_row + 18] = f_res

            c_offset = 18 * k
            for r in range(18):
                for c in range(36):
                    val = f_jac[r, c]
                    if val != 0.0:
                        jac_data.append(val)
                        jac_rows.append(curr_row + r)
                        jac_cols.append(c_offset + c)
            curr_row += 18

        jacobian = sp.csr_matrix((jac_data, (jac_rows, jac_cols)), shape=(row_count, col_count))
        return res, jacobian

    def _measurement_residual(
        self,
        T: np.ndarray,
        W: np.ndarray,
        T_meas: np.ndarray,
        W_meas: np.ndarray,
    ) -> np.ndarray:
        # T_meas / T == T_meas * inv(T)
        step_T = T_meas @ inv_se3(T)
        err_pose = log_se3(step_T) / self.options["poseStd"]
        err_twist = (W - W_meas) / self.options["twistStd"]
        return np.concatenate([err_pose, err_twist])

    def _linearize_measurement(
        self,
        T: np.ndarray,
        W: np.ndarray,
        T_meas: np.ndarray,
        W_meas: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        res = self._measurement_residual(T, W, T_meas, W_meas)
        jac = np.zeros((12, 18), dtype=float)
        eps = float(self.options["finiteDifferenceStep"])

        for col in range(6):
            pert = np.zeros(6, dtype=float)
            pert[col] = eps
            T_plus = exp_se3(pert) @ T
            plus = self._measurement_residual(T_plus, W, T_meas, W_meas)
            jac[:, col] = (plus - res) / eps

        jac[6:12, 6:12] = np.diag(1.0 / self.options["twistStd"])
        return res, jac

    def _paper_prior_residual(
        self,
        Ti: np.ndarray,
        Wi: np.ndarray,
        Di: np.ndarray,
        Tj: np.ndarray,
        Wj: np.ndarray,
        Dj: np.ndarray,
        dt: float,
    ) -> np.ndarray:
        eta = log_se3(Tj @ inv_se3(Ti))
        local_wj, local_dj = self._local_kinematics(eta, Wj, Dj)
        gamma_i = np.concatenate([np.zeros(6), Wi, Di])
        gamma_j = np.concatenate([eta, local_wj, local_dj])
        Phi, _ = self.transition(dt, np.eye(6))
        return gamma_j - Phi @ gamma_i

    def _prior_whitened_residual(
        self,
        Ti: np.ndarray,
        Wi: np.ndarray,
        Di: np.ndarray,
        Tj: np.ndarray,
        Wj: np.ndarray,
        Dj: np.ndarray,
        dt: float,
    ) -> np.ndarray:
        raw_res = self._paper_prior_residual(Ti, Wi, Di, Tj, Wj, Dj, dt)
        _, Q = self.transition(dt, self.options["jerkSpectralDensity"])
        L = la.cholesky(Q, lower=True)
        return la.solve_triangular(L, raw_res, lower=True)

    def _linearize_prior(
        self,
        Ti: np.ndarray,
        Wi: np.ndarray,
        Di: np.ndarray,
        Tj: np.ndarray,
        Wj: np.ndarray,
        Dj: np.ndarray,
        dt: float,
    ) -> Tuple[np.ndarray, np.ndarray]:
        raw_res = self._paper_prior_residual(Ti, Wi, Di, Tj, Wj, Dj, dt)
        raw_jac = np.zeros((18, 36), dtype=float)
        eps = float(self.options["finiteDifferenceStep"])

        for col in range(6):
            pert = np.zeros(6, dtype=float)
            pert[col] = eps

            # Perturb Ti pose
            plus_ti = self._paper_prior_residual(exp_se3(pert) @ Ti, Wi, Di, Tj, Wj, Dj, dt)
            raw_jac[:, col] = (plus_ti - raw_res) / eps

            # Perturb Tj pose
            plus_tj = self._paper_prior_residual(Ti, Wi, Di, exp_se3(pert) @ Tj, Wj, Dj, dt)
            raw_jac[:, 18 + col] = (plus_tj - raw_res) / eps

            # Perturb Wj twist
            Wj_plus = np.copy(Wj)
            Wj_plus[col] += eps
            plus_wj = self._paper_prior_residual(Ti, Wi, Di, Tj, Wj_plus, Dj, dt)
            raw_jac[:, 24 + col] = (plus_wj - raw_res) / eps

        eta = log_se3(Tj @ inv_se3(Ti))
        J_eta, _ = self._left_jacobian_and_derivative(eta, np.zeros(6))
        J_inv = np.linalg.inv(J_eta)

        I6 = np.eye(6, dtype=float)
        raw_jac[0:6, 6:12] = -dt * I6
        raw_jac[0:6, 12:18] = -0.5 * (dt**2) * I6
        raw_jac[6:12, 6:12] = -I6
        raw_jac[6:12, 12:18] = -dt * I6
        raw_jac[12:18, 12:18] = -I6
        raw_jac[12:18, 30:36] = J_inv

        _, Q = self.transition(dt, self.options["jerkSpectralDensity"])
        L = la.cholesky(Q, lower=True)
        whitened_res = la.solve_triangular(L, raw_res, lower=True)
        whitened_jac = la.solve_triangular(L, raw_jac, lower=True)
        return whitened_res, whitened_jac

    def _retract(
        self,
        T: np.ndarray,
        W: np.ndarray,
        D: np.ndarray,
        delta: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        K = W.shape[1]
        T_new = np.zeros_like(T)
        W_new = np.zeros_like(W)
        D_new = np.zeros_like(D)

        for k in range(K):
            offset = 18 * k
            T_new[:, :, k] = exp_se3(delta[offset : offset + 6]) @ T[:, :, k]
            W_new[:, k] = W[:, k] + delta[offset + 6 : offset + 12]
            D_new[:, k] = D[:, k] + delta[offset + 12 : offset + 18]

        return T_new, W_new, D_new

    def _interpolate(
        self,
        Ti: np.ndarray,
        Wi: np.ndarray,
        Di: np.ndarray,
        Tj: np.ndarray,
        Wj: np.ndarray,
        Dj: np.ndarray,
        dt: float,
        tau: float,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        Qc = self.options["jerkSpectralDensity"]
        Phi_tau, Q_tau = self.transition(tau, Qc)
        Phi_dt, Q_dt = self.transition(dt, Qc)
        Phi_rem, _ = self.transition(dt - tau, Qc)

        # Omega = Q_tau @ Phi_rem.T / Q_dt
        Omega = np.linalg.solve(Q_dt.T, Phi_rem @ Q_tau.T).T
        Lambda = Phi_tau - Omega @ Phi_dt

        eta = log_se3(Tj @ inv_se3(Ti))
        local_wj, local_dj = self._local_kinematics(eta, Wj, Dj)

        gamma_i = np.concatenate([np.zeros(6), Wi, Di])
        gamma_j = np.concatenate([eta, local_wj, local_dj])
        gamma = Lambda @ gamma_i + Omega @ gamma_j

        xi = gamma[0:6]
        xi_dot = gamma[6:12]
        xi_ddot = gamma[12:18]

        J_xi, J_xi_dot = self._left_jacobian_and_derivative(xi, xi_dot)
        W = J_xi @ xi_dot
        D = J_xi @ xi_ddot + J_xi_dot @ xi_dot
        T = exp_se3(xi) @ Ti
        return T, W, D

    def _left_jacobian_and_derivative(
        self,
        xi: np.ndarray,
        xi_dot: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Compute SE(3) left Jacobian J_l(xi) and its exact time derivative Jdot_l(xi, xi_dot)."""
        ad_xi = ad_twist(xi)
        ad_xi_dot = ad_twist(xi_dot)

        power = np.eye(6, dtype=float)
        power_deriv = np.zeros((6, 6), dtype=float)
        J = np.eye(6, dtype=float)
        J_dot = np.zeros((6, 6), dtype=float)
        coeff = 1.0

        for order in range(1, 25):
            power_deriv = power_deriv @ ad_xi + power @ ad_xi_dot
            power = power @ ad_xi
            coeff /= (order + 1)
            J += coeff * power
            J_dot += coeff * power_deriv
            if (
                np.linalg.norm(coeff * power, "fro") < 1e-16
                and np.linalg.norm(coeff * power_deriv, "fro") < 1e-16
            ):
                break

        return J, J_dot

    def _local_kinematics(
        self,
        xi: np.ndarray,
        W: np.ndarray,
        D: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        J, _ = self._left_jacobian_and_derivative(xi, np.zeros(6))
        xi_dot = np.linalg.solve(J, W)
        _, J_dot = self._left_jacobian_and_derivative(xi, xi_dot)
        xi_ddot = np.linalg.solve(J, D - J_dot @ xi_dot)
        return xi_dot, xi_ddot

    def _project_state_to_paper(
        self,
        H: np.ndarray,
        V: np.ndarray,
        A: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        return inv_se3(H), -np.asarray(V, dtype=float), -np.asarray(A, dtype=float)

    def _paper_state_to_project(
        self,
        T: np.ndarray,
        W: np.ndarray,
        D: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        return inv_se3(T), -np.asarray(W, dtype=float), -np.asarray(D, dtype=float)

    def _paper_arrays_to_project(
        self,
        T: np.ndarray,
        W: np.ndarray,
        D: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        K = T.shape[2]
        H = np.zeros((4, 4, K), dtype=float)
        for k in range(K):
            H[:, :, k] = inv_se3(T[:, :, k])
        return H, -np.copy(W), -np.copy(D)

    def _final_relative_decrease(self, costs: List[float]) -> float:
        if len(costs) < 2:
            return float("nan")
        return float((costs[-2] - costs[-1]) / max(1.0, costs[-2]))
