# Numerical-results diagnostic report

## Corrected issues

- Connection residual had the wrong sign and used the reference twist instead of s; a nonzero-error regression test exposed it. Both wrenches now use the same sampled state/reference, including desired acceleration.
- Previous release peak was computed before release; previous post window started at 12 s. Windows now split exactly at 10 s.
- Attitude metrics are degrees, not unlabeled radians. Mixed force/torque effort is removed.
- Dwell checks formerly excluded the endpoint. Numerical reaching now uses the weighted s norm.
- Previous reaching experiment started on s=0 and had a vacuous zero bound. Isolated test starts with a nonzero twist and uses true inertia to recompute energy.
- Logged s and energy previously held stale controller samples; now evaluated at each plant sample. Estimates are logged before the next update. Physical margins are computed for both estimators.
- The payload-release comparison runs nominal, Euclidean, and Natural/Bregman controllers with the LC realization. The separate nominal validation protocol is the only section that compares LC and RB. The paper runner does not optimize gains; saved estimator rates are used as configured.
- PyBullet previously recomputed inertia from collision geometry (Ixx approximately 0.0800 instead of 0.0409). URDF inertia loading, principal-inertia/body-frame transforms, body-origin wrench application and payload attachment frames are corrected and regression-tested. Gain entries without current optimization metadata are configuration values, not optimality evidence for the corrected plant.
- The relative connection residual is ill-conditioned near zero differences. Absolute residual is checked at EVERY sample with tolerance 1e-12+1e-10*signal norm; relative diagnostics exclude signal norms <=1e-6.

## Publication qualifications

- Connection-plot y-axis titles omit [1] for readability, but both norms are dimensionless: torque is divided by 1 N m and force by 1 N before taking the Euclidean norm. This is not mixed-unit wrench effort; see metric_definitions.md. The identity residual is theoretically zero at all times; its computed roundoff-level values remain on a logarithmic scale.

- For the payload-release comparison, each controller uses its committed LC gain set (nominal-LC, Euclidean-LC, Bregman-LC). The nominal connection/reaching study is a separate no-payload protocol and is the only LC/RB comparison.
- `run_paper_sim` loads the gain registry and does not execute PSO. Estimator rates are reported as configured; the saved values do not establish a fair estimator-gain tuning comparison.
- The Known-inertia controller receives the true loaded inertia before release and the true bare-vehicle inertia after release. It is included as a model-knowledge reference; the adaptive estimators do not receive this parameter switch. The separate nominal reaching test has no payload release.
- The matched LC/RB closed-loop study is a separate test from the same-state connection identity. Its protocol file records equal plant, initial state, reference samples, and gains; only the Coriolis realization changes. Small off-manifold differences are expected and neither realization is ranked.
- No rotor allocation or actuator limits exist in this ideal wrench-actuated model. These plots do not establish hardware feasibility.
- Payload release violates the constant-parameter assumption at the jump. Adaptive asymptotic theory does not assert finite-time reaching or parameter convergence.
- Bregman stepping uses an SPD-preserving exponential update with numerical eigenvalue/exponent safeguards; it is not exact continuous-time integration.
- Identical repeated trials and all old flat figures/tables are superseded and must not be cited.
- FAILED reaching figures are diagnostics only; numerical threshold crossing is not exact finite-time convergence.
- Payload time histories use 0--30 s. Every figure in 04-nominal-validation uses the 4x connection sensitivity experiment on source time 10--30 s. `T_obs` is the first saved sample from which weighted s stays <=1e-4 through source time 30 s; LC and RB are assessed independently. The connection residual remains logarithmic. See connection_sensitivity_diagnostics.md for the experiment and numerical limitations.
- The nominal transverse-energy integral's relative numerical residual is retained in the reaching summary. Report the threshold-and-dwell bound check as numerical evidence, not as a pointwise reproduction of the continuous-time energy identity.

## Current checks

- optimization_run: False
- fair_adaptation_retuned: False
- adaptation_gain_tuning: saved configuration values; no retuning performed by run_paper_sim
- repeatability: omitted: deterministic identical trials are not repeatability evidence
- fair_adaptation_retuning_pending: True
- Known-inertia controller: {'failure': None, 'end_time': 30.0}
- Euclidean adaptive controller: {'failure': None, 'end_time': 30.0}
- Natural/Bregman adaptive controller: {'failure': None, 'end_time': 30.0}
- connection_realization_lc_failure: None
- connection_realization_rb_failure: None
- connection_realization_protocol_passed: True
- nominal_reaching_passed: True
- nominal_replay_duration_s: 20.0
- nominal_energy_residual_relative_to_initial: 0.003035680224515749
- connection_identity_passed: True
- nominal_refinement_passed: True
- nominal_experiment: connection-sensitivity
- nominal_later_threshold_departure: True
- nominal_persistent_invariance_passed: True
- nominal_lc: {'failure': None, 'end_time': 30.0}
- euclidean_lc: {'failure': None, 'end_time': 30.0}
- bregman_lc: {'failure': None, 'end_time': 30.0}
