# Connection sensitivity metrics and diagnostics

Full run: lemniscate source time 10-30 s. Initial twist perturbation is 4x the original; pose starts at the desired pose. This is not a physical payload-drop experiment.

RMSE is sqrt(mean(squared error norm)); attitude uses geodesic degrees. Persistent reaching is the first sample with weighted s <= 1e-4 that stays below the threshold through source time 30 s. LC and RB reaching times are assessed independently.

Separation compares positions in metres, relative rotation angle in degrees, and force/torque commands rotated into inertial axes. Torque is the commanded free couple about each body origin; it excludes position-cross-force moments about a common spatial origin. Independent-trajectory command differences are not the same-state connection identity. The identity and command replay checks evaluate every logged sample. Identity norms scale torque by 1 N m and force by 1 N; relative residuals exclude signals <=1e-6.

Refinement compares 2 ms and 1 ms integration with identical gains through the full interval. JSON reports trace differences and reaching times without assuming agreement. Full-run peaks and times may lie outside the focused view. The CSV repeats pair separation peaks for both controller rows; they are not per-controller performance measures.

- lc: {'T_obs_elapsed_s': 11.218, 'T_obs_source_s': 21.218, 'persistent_threshold': 0.0001, 'persistent_final_source_time_s': 30.0, 'persistent_invariance_passed': True, 'initial_s': array([ 1.2, -0.8,  0.4,  1.6, -0.8,  1.2]), 'initial_weighted_s': 4.125340755477055, 'identity_max_residual': 3.873740852519421e-14, 'identity_rms_residual': 6.787557543436354e-15, 'identity_max_relative_residual': 8.221774259949764e-09, 'identity_passed': True, 'maximum_command_replay_error': 0.0, 'command_replay_passed': True}
- rb: {'T_obs_elapsed_s': 11.208, 'T_obs_source_s': 21.208, 'persistent_threshold': 0.0001, 'persistent_final_source_time_s': 30.0, 'persistent_invariance_passed': True, 'initial_s': array([ 1.2, -0.8,  0.4,  1.6, -0.8,  1.2]), 'initial_weighted_s': 4.125340755477055, 'identity_max_residual': 3.648247535036918e-14, 'identity_rms_residual': 6.739704046929729e-15, 'identity_max_relative_residual': 9.877326923013124e-09, 'identity_passed': True, 'maximum_command_replay_error': 0.0, 'command_replay_passed': True}

Refinement relative trace differences: {'Position separation [m]': 0.1670349572804272, 'Attitude separation [deg]': 0.042559519233301986, 'Force difference [N]': 0.115157457253433, 'Torque difference [N m]': 0.17686371016817884}


- Persistent reaching is the first saved sample after which the weighted norm remains <=1e-4 through source time 30 s.
- Two integration steps measure sensitivity, not established numerical convergence.
- LC and RB reaching times are assessed independently and may differ.
- Different configurations at reaching may retain position separation while following the same reduced vector field.
