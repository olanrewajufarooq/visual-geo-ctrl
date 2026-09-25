# Numerical-results diagnostic report

## Corrected issues

- Connection residual had the wrong sign and used the reference twist instead of s; a nonzero-error regression test exposed it. Both wrenches now use the same sampled state/reference, including desired acceleration.
- Previous release peak was computed before release; previous post window started at 12 s. Windows now split exactly at 10 s.
- Attitude metrics are degrees, not unlabeled radians. Mixed force/torque effort is removed.
- Dwell checks formerly excluded the endpoint. Numerical reaching now uses the weighted s norm.
- Previous reaching experiment started on s=0 and had a vacuous zero bound. Isolated test starts with a nonzero twist and uses true inertia to recompute energy.
- Logged s and energy previously held stale controller samples; now evaluated at each plant sample. Estimates are logged before the next update. Physical margins are computed for both estimators.
- Tracking gains were independently tuned; all primary comparisons now share one saved optimized gain set.
- PyBullet previously recomputed inertia from collision geometry (Ixx approximately 0.0800 instead of 0.0409). URDF inertia loading, principal-inertia/body-frame transforms, body-origin wrench application and payload attachment frames are corrected and regression-tested. Existing optimized gains were obtained on that old plant and are not optimality evidence for the corrected plant.
- The relative connection residual is ill-conditioned near zero differences. Absolute residual is checked at EVERY sample with tolerance 1e-12+1e-10*signal norm; relative diagnostics exclude signal norms <=1e-6.

## Publication qualifications

- Connection-plot y-axis titles omit [1] for readability, but both norms are dimensionless: torque is divided by 1 N m and force by 1 N before taking the Euclidean norm. This is not mixed-unit wrench effort; see metric_definitions.md. The identity residual is theoretically zero at all times; its computed roundoff-level values remain on a logarithmic scale.

- Adaptation-only re-optimization is NOT run. Existing gamma and gamma_B are provisional, not a jointly fair optimized comparison. Do not claim optimality or estimator superiority from these runs.
- The fixed-model controller is excluded from adaptive figures, metrics and default runs. The separate nominal theory tests use exact known inertia, not an adaptive estimator.
- No rotor allocation or actuator limits exist in this ideal wrench-actuated model. These plots do not establish hardware feasibility.
- Payload release violates the constant-parameter assumption at the jump. Adaptive asymptotic theory does not assert finite-time reaching or parameter convergence.
- Bregman stepping uses an SPD-preserving exponential update with numerical eigenvalue/exponent safeguards; it is not exact continuous-time integration.
- Identical repeated trials and all old flat figures/tables are superseded and must not be cited.
- FAILED reaching figures are diagnostics only; numerical threshold crossing is not exact finite-time convergence.
- All payload time histories use 0--30 s. Isolated reaching and connection-equivalence figures share a focused reaching-interval view. The connection residual remains logarithmic and its x-axis measures time since initialization. Nominal runs have no payload-release marker because no release occurs.

## Current checks

- optimization_run: False
- fair_adaptation_retuning_pending: True
- repeatability: omitted: deterministic identical trials are not repeatability evidence
- Euclidean adaptive controller: {'failure': None, 'end_time': 30.0}
- Natural/Bregman adaptive controller: {'failure': None, 'end_time': 30.0}
- nominal_reaching_passed: True
- connection_identity_passed: True
- nominal_refinement_passed: True
