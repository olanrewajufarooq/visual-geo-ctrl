# SE(3) Replay-Trajectory Post-Processing

## Goal

Generate a smooth replay reference whose pose, body twist, and body
acceleration describe the same rigid-body motion:

$$
\dot{H}(t) = H(t)\widehat{V}(t),
\qquad
A(t) = \dot{V}(t).
$$

The post-processor supports an offline **white-noise-on-jerk (WNOJ) batch
smoother on $SE(3)$**. It estimates one continuous trajectory

$$
x(t) = \{H(t), V(t), A(t)\},
$$

where

$$
H = \begin{bmatrix} R & p \\ 0 & 1 \end{bmatrix},
\qquad
V = \begin{bmatrix} \omega_b \\ v_b \end{bmatrix},
\qquad
A = \begin{bmatrix} \alpha_b \\ a_b \end{bmatrix}.
$$

## WNOJ formulation

The prior models jerk as white Gaussian noise in local coordinates:

$$
\frac{d^3\xi(t)}{dt^3} = w(t),
\qquad
w(t) \sim \mathcal{GP}\!\left(0,Q_c\delta(t-t')\right).
$$

Its mean behavior is constant acceleration. Given measured MoCap poses
$H_k^{\mathrm{meas}}$ and the dataset's world-frame twists
$V_{w,k}^{\mathrm{meas}}$, estimate
the knot states by minimizing

$$
\begin{aligned}
\min_{\{H_k,V_k,A_k\}} \sum_k &\left\|
\log\!\left(H_k^{-1}H_k^{\mathrm{meas}}\right)^\vee
\right\|^2_{\Sigma_H^{-1}}
+ \left\|V_{w,k}-V_{w,k}^{\mathrm{meas}}\right\|^2_{\Sigma_V^{-1}} \\
&+ \sum_k
\left\|r_{\mathrm{WNOJ}}(x_k,x_{k+1})\right\|^2_{Q_k^{-1}}.
\end{aligned}
$$

The final term couples neighboring pose, twist, and acceleration states while
penalizing implausible jerk. The current replay implementation is a practical
fixed-interval local-coordinate Kalman/RTS approximation to this objective. It
uses world-frame derivatives internally, unwraps incremental rotations, and
converts the result to body coordinates at output; it is not yet the paper's
full nonlinear batch solver with all $SE(3)$ Jacobians.

## Replay conventions

The CSV world-frame velocity channels are retained during WNOJ smoothing and
converted only at output:

$$
v_b=R^\mathsf{T}v_w,
\qquad
\omega_b=R^\mathsf{T}\omega_w.
$$

The smoother exports $H_d$, $V_d$, and $A_d$ in the controller's body-twist
convention. The translational transport term is applied analytically when
converting world acceleration to body acceleration.

Tune the pose covariance $\Sigma_H$, twist covariance $\Sigma_V$, and separate
rotational/translational jerk covariance $Q_c$ against pose deviation,
geometric consistency, acceleration spikes, and closed-loop tracking.

## References and baseline

The primary reference is Tang, Yoon, and Barfoot, “A White-Noise-on-Jerk Motion
Prior for Continuous-Time Trajectory Estimation on SE(3),” *IEEE Robotics and
Automation Letters*, 4(2):594–601, 2019, DOI
[10.1109/LRA.2019.2891492](https://doi.org/10.1109/LRA.2019.2891492).

Download the open-access author version from
[arXiv:1809.06518](https://arxiv.org/abs/1809.06518) using **Download PDF**, or
directly from <https://arxiv.org/pdf/1809.06518>. The publisher version is
available through the DOI when IEEE Xplore access is available.
