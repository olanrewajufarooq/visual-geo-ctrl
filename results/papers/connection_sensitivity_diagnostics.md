# Connection sensitivity metrics and diagnostics

Full run: lemniscate source time 10–30 s. Initial twist perturbation is 4x the original; pose starts at the desired pose. This is not a physical payload-drop experiment.

RMSE is sqrt(mean(squared error norm)); attitude uses geodesic degrees. Reaching is the first sample satisfying weighted s <= 0.001 through a complete 0.5 s dwell. The focus covers the later reaching time, dwell, and 0.25 s margin, rounded upward to 0.25 s. Missing reaching uses the full interval. Later departures are reported in the JSON.

Separation compares positions in metres, relative rotation angle in degrees, and force/torque commands rotated into inertial axes. Torque is the commanded free couple about each body origin; it excludes position-cross-force moments about a common spatial origin. Independent-trajectory command differences are not the same-state connection identity. The identity and command replay checks evaluate every logged sample. Identity norms scale torque by 1 N m and force by 1 N; relative residuals exclude signals <=1e-6.

Refinement compares 2 ms and 1 ms integration with identical gains through the focused interval. JSON reports trace differences and reaching times without assuming agreement. Full-run peaks and times may lie outside the focused view. The CSV repeats pair separation peaks for both controller rows; they are not per-controller performance measures.

- lc: {'elapsed_reaching_s': 2.15, 'absolute_reaching_s': 12.15, 'initial_s': array([ 1.2, -0.8,  0.4,  1.6, -0.8,  1.2]), 'initial_weighted_s': 4.125340755477056, 'later_threshold_departure': True, 'first_later_departure_s': 13.534, 'identity_max_residual': 3.528697999707463e-14, 'identity_rms_residual': 6.804174429860206e-15, 'identity_max_relative_residual': 8.267960340840283e-09, 'identity_passed': True, 'maximum_command_replay_error': 0.0, 'command_replay_passed': True}
- rb: {'elapsed_reaching_s': 2.15, 'absolute_reaching_s': 12.15, 'initial_s': array([ 1.2, -0.8,  0.4,  1.6, -0.8,  1.2]), 'initial_weighted_s': 4.125340755477056, 'later_threshold_departure': True, 'first_later_departure_s': 13.546, 'identity_max_residual': 3.4322769733662464e-14, 'identity_rms_residual': 6.72664580274904e-15, 'identity_max_relative_residual': 9.870375341102018e-09, 'identity_passed': True, 'maximum_command_replay_error': 0.0, 'command_replay_passed': True}

Refinement relative trace differences: {'Position separation [m]': 0.16703495728042522, 'Attitude separation [deg]': 0.042559519233301986, 'Force difference [N]': 0.07929808049376094, 'Torque difference [N m]': 0.03489903143363157}


- Observed reaching certifies a finite sampled dwell only; later threshold departures are reported.
- Two integration steps measure sensitivity, not established numerical convergence.
- Fine-run later-departure checks cover only the focused interval, not the full 10–30 s run.
- Different configurations at reaching may retain position separation while following the same reduced vector field.
