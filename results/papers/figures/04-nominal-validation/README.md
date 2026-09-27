# Nominal connection sensitivity figures

## Nominal figures: connection sensitivity experiment

All figures in figures/04-nominal-validation use the same 4x sensitivity pair. The previous 1x experiment is superseded in this folder. Initial pose equals the desired lemniscate pose at source time 10 s; initial twist error is [1.2,-0.8,0.4] rad/s and [1.6,-0.8,1.2] m/s. The plant has exact bare-vehicle inertia, no adaptation, and no physical payload release. LC/RB gains are identical. Full tracking and error figures cover source time 10–30 s. Reaching, same-state identity, and sensitivity detail figures share the observed-reaching window in ../../metadata/connection_sensitivity_protocol.json. T_obs in summaries is elapsed time from source time 10 s; its plotted location is 10+T_obs. The conservative bound is also an elapsed duration.

See ../../metadata/connection_sensitivity_diagnostics.md for physical separation definitions, refinement sensitivity and later threshold departures. Finite sampled dwell does not establish permanent sliding, and the two-step comparison does not establish numerical convergence.
