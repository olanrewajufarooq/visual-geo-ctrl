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
- Historical optimizer records contain different tracking gains for Euclidean and Natural/Bregman variants. The published runs overwrite those common gains identically, but the records do not establish a common objective, trajectory, initial estimate, duration, constraints, and weights for gamma versus gamma_B.
- The Known-inertia controller receives the true loaded inertia before release and the true bare-vehicle inertia after release. It is included as a model-knowledge reference; the adaptive estimators do not receive this parameter switch. The separate nominal reaching test has no payload release.
- The matched LC/RB closed-loop study is a separate test from the same-state connection identity. Its protocol file records equal plant, initial state, reference samples, and gains; only the Coriolis realization changes. Small off-manifold differences are expected and neither realization is ranked.
- No rotor allocation or actuator limits exist in this ideal wrench-actuated model. These plots do not establish hardware feasibility.
- Payload release violates the constant-parameter assumption at the jump. Adaptive asymptotic theory does not assert finite-time reaching or parameter convergence.
- Bregman stepping uses an SPD-preserving exponential update with numerical eigenvalue/exponent safeguards; it is not exact continuous-time integration.
- Identical repeated trials and all old flat figures/tables are superseded and must not be cited.
- FAILED reaching figures are diagnostics only; numerical threshold crossing is not exact finite-time convergence.
- All payload time histories use 0--30 s. The nominal tests replay the lemniscate starting from its payload-release phase (10 s). The connection-identity figure uses the original lemniscate clock and begins at 10 s; reaching times remain measured from initialization. The available replay segment ends at 20.882 s, so nominal closed-loop plots stop there rather than clamping a desired state. Reaching and connection-equivalence figures share a focused reaching-interval view. The connection residual remains logarithmic.
- The nominal transverse-energy integral's relative numerical residual is retained in the reaching summary. Report the threshold-and-dwell bound check as numerical evidence, not as a pointwise reproduction of the continuous-time energy identity.

## Current checks

- optimization_run: False
- fair_adaptation_retuning_pending: True
- repeatability: omitted: deterministic identical trials are not repeatability evidence
- Known-inertia controller: {'failure': None, 'end_time': 30.0}
- Euclidean adaptive controller: {'failure': None, 'end_time': 30.0}
- Natural/Bregman adaptive controller: {'failure': None, 'end_time': 30.0}
- connection_realization_lc_failure: None
- connection_realization_rb_failure: None
- connection_realization_protocol_passed: True
- nominal_reaching_passed: True
- nominal_replay_duration_s: 20.882
- nominal_energy_residual_relative_to_initial: 0.025379831983687488
- connection_identity_passed: True
- nominal_refinement_passed: True
