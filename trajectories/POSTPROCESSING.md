# SE(3) Replay-Trajectory Post-Processing

## Goal

The replay reference must describe one rigid-body motion:

$$
\dot{H}(t)=H(t)\widehat{V}(t),
\qquad
A(t)=\dot{V}(t),
$$

where $H$ maps body coordinates to world coordinates and $V,A\in\mathbb{R}^6$
are body-coordinate twist and twist derivative.

The CSV stores linear and angular velocity in the world frame. Before fitting,
the processor converts them using

$$
v_b=R^\mathsf{T}v_w,
\qquad
\omega_b=R^\mathsf{T}\omega_w.
$$

The IMU `accel_*` channels are specific force and are not used as kinematic
acceleration.

## Nonlinear WNOJ batch smoother

The implementation follows Tang, Yoon, and Barfoot's white-noise-on-jerk
motion prior. Their pose convention is obtained exactly from the project state:

$$
T=H^{-1},
\qquad
\varpi=-V,
\qquad
\dot{\varpi}=-A,
$$

so that

$$
\dot{T}=\widehat{\varpi}T.
$$

For adjacent knots $i$ and $j$, define

$$
\eta_{ij}=\log\!\left(T_jT_i^{-1}\right)^\vee,
\qquad
J_{ij}^{-1}=J_\ell(\eta_{ij})^{-1}.
$$

The local state at the right knot is

$$
\gamma_j=
\begin{bmatrix}
\eta_{ij}\\
\dot{\xi}_j\\
\ddot{\xi}_j
\end{bmatrix},
$$

where the implementation evaluates the complete left-Jacobian derivative:

$$
\dot{\xi}_j=J_\ell(\eta_{ij})^{-1}\varpi_j,
\qquad
\ddot{\xi}_j=J_\ell(\eta_{ij})^{-1}
\left(\dot{\varpi}_j-\dot{J}_\ell(\eta_{ij})\dot{\xi}_j\right).
$$

This replaces the paper's first-order approximation of $\dot{J}_\ell$ so that
the exported acceleration is the derivative of the exported twist during
interpolation, including at knot boundaries.

and the left-knot state is

$$
\gamma_i=
\begin{bmatrix}
0\\ \varpi_i\\ \dot{\varpi}_i
\end{bmatrix}.
$$

The WNOJ prior residual is

$$
r_{ij}=\gamma_j-\Phi(\Delta t_{ij})\gamma_i,
$$

with

$$
\Phi(\Delta t)=
\begin{bmatrix}
I & \Delta t I & \tfrac{1}{2}\Delta t^2 I\\
0 & I & \Delta t I\\
0 & 0 & I
\end{bmatrix}.
$$

Pose and twist measurements are combined with these neighboring-knot priors in
a nonlinear least-squares problem. Each factor touches at most two 18-variable
knot states, so the implementation assembles and solves sparse normal equations.
Levenberg--Marquardt trial steps are accepted only when they reduce total cost.

The default knot spacing is 0.01 s. The controller's original 500 Hz output grid
is recovered with the paper's Gaussian-process interpolation equations, then
converted back to $(H,V,A)$. Optimization status, cost history, sparse-system
size, final step and cost decrease, and termination reason are stored in each
artifact's diagnostics. Reaching a fixed iteration limit emits a warning and is
recorded as `converged = false`; it is never reported as numerical convergence.

## Processing methods

`process_trajectories.m` supports two method names:

- `wnoj`: nonlinear $SE(3)$ WNOJ batch smoothing.
- `poly`: local-polynomial velocity differentiation retained as a baseline.

Set `trajIds = {}` in the script to process the complete manifest, or list the
manifest keys that should be regenerated.

## Reference

Tang, Yoon, and Barfoot, “A White-Noise-on-Jerk Motion Prior for
Continuous-Time Trajectory Estimation on SE(3),” *IEEE Robotics and Automation
Letters*, 4(2):594–601, 2019,
[doi:10.1109/LRA.2019.2891492](https://doi.org/10.1109/LRA.2019.2891492).

The author manuscript can be downloaded from
[arXiv:1809.06518](https://arxiv.org/abs/1809.06518) with **Download PDF**, or
directly from <https://arxiv.org/pdf/1809.06518>.
