# Metric definitions

Uniform plant samples include t=0 and the final sample. Pre-release means t<10 s;
post-release means t>=10 s. RMSE = sqrt(mean(error_norm**2)) over that window.
Position norm uses inertial positions in metres. Attitude error is
acos(clip((trace(Rd.T R)-1)/2,-1,1)), converted to degrees. Peaks are window maxima.
No transient is removed. Full-run force/torque RMS = sqrt(mean(norm**2)); peaks
are max(norm), separately in N and N m. Commands are the actual zero-order-held wrenches.

Recovery: first sampled t>=10 for which position<=0.05 m AND geodesic angle<=5 deg
at every sample through the first sample at or after t+1 s (inclusive).
Report absolute time and duration t-10. Blank means not observed with a complete dwell.
This is sampled dwell evidence, not a guarantee between samples or for all future time.

T_obs: first saved sample with sqrt(s.T Lambda_s s)<=1e-4 at every sample
through source time 30 s. The complete horizon is required; incomplete logs
do not qualify. LC and RB times are assessed independently.
Lambda_s is the configured transverse metric. The bound uses true I and
actual s(0), never estimated energy. The nominal controller is continuous
in theory but evaluated at finite sample rate.

Physical margin: smallest eigenvalue of Jhat directly (Bregman) or pseudo_from_pi
(Euclidean), with full-run and post-release minima and nonpositive flag.
Pseudo-inertia combines kg, kg m, kg m^2; eigenvalues are coordinate-scaled SI
certificates, not a scalar with one physical unit. Likewise the weighted s norm
uses the specified design metric and therefore has no single physical unit.

RPY is xyz Euler visualization, unwrapped independently in time and aligned by
integer 360-degree offsets initially; geodesic error is used for all quantitative claims.
Desired velocity in each actual body is Ad_(He^-1) Vd, including the translational
adjoint term. These transported references differ slightly between controllers;
the black dashed curves are labelled "Transported reference" and show each,
not an incorrect shared raw Vd.

## Connection-equivalence norm and axis labels

For a wrench w = [tau; f], the dimensionless scaled norm is

```text
||w||_* = sqrt(sum_i (tau_i / (1 N m))^2 + sum_i (f_i / (1 N))^2).
```

The same scaling applies to the wrench difference, K_RB(V)s, and the identity
residual r_K. With unit reference scales in SI, this equals the Euclidean norm
of the stored six numerical components. It is a visualization/validation norm,
not an energy or control-effort measure. Other reference scales change the
magnitude but not whether the identity residual is zero.

Both y-axes are dimensionless. Their titles are "Scaled wrench norm" and
"Scaled residual norm"; [1] is intentionally omitted for readability, not
replaced by N or N m. Force and torque metrics elsewhere retain separate units.

Theory predicts r_K = W_RB - W_LC + K_RB(V)s = 0 at every time, including off
the sliding manifold. The wrench difference itself need only vanish when s=0.
The computed small residual is consistent with floating-point roundoff; the
logarithmic panel retains these values rather than setting them to zero.
T_obs uses the 1e-4 persistence threshold through source time 30 s. It does
not determine when the algebraic identity becomes valid; that identity holds
at every sample.

Relative residual uses max(norm(left),norm(right)) only above 1e-6.
Separate maximum force and torque residuals are also saved.

## Center-of-mass and principal-inertia diagnostics

The additional estimator figures reconstruct body-frame c = h/m [m] and
J_c = J_b - m ((c.T c) I - c c.T) [kg m^2] from estimatePi.
True references use activePlantPi at the same logged samples, including the
payload jump at 10 s. Principal moments are the ascending eigenvalues of J_c,
not body-origin diagonal entries or eigenvalues of the pseudo-inertia.
Their indices denote magnitude ordering, not continuously tracked principal axes.

Nonfinite parameter samples, nonpositive mass, or nonfinite reconstructions
produce warnings and gaps. Finite negative principal moments remain visible;
there is no filtering or projection to physical values. These are estimator
trajectories, not evidence of parameter convergence or sufficient excitation.
The existing pseudo-inertia certificate remains the physical-consistency test.

Exports: estimated_center_of_mass.pdf/png and estimated_principal_inertia.pdf/png
in figures/02-physical-consistency. Both show 0–30 s with a 10 s release marker,
Euclidean blue dash-dot, Natural/Bregman solid green, and true values black dashed.
