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

## Artifact-generation status

The figures are generated using `run/run_paper_sim_figures.py` with the repository's saved optimized gains. Simulations cover 0–30 s with payload release at 10 s.

