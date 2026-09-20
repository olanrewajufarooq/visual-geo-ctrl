# Architecture

`agc.sim.runScenario` is the only closed-loop execution path. A scenario carries true plant parameters, initial state and estimate, replay sampler, controller gains, and three fixed rates. The runner sends held body wrenches to a Robotics System Toolbox floating-body plant and logs paper-facing diagnostics.

Gravity is explicit at the interface: `controller.gravity = [0;0;9.81]` follows the paper's compensation-wrench convention, while `plantGravity = [0;0;-9.81]` is the physical world acceleration supplied to Robotics System Toolbox. They must not be conflated.

Default theory scenarios initialize nominal control at the true inertial parameters. Euclidean and Bregman adaptation instead share one deterministic 2% affine-invariant pseudo-inertia perturbation, represented as \hat\pi for Euclidean adaptation and \hat{\mathcal J} for Bregman adaptation.

`agc.paper` is intentionally function-based. It exposes the two Coriolis actions, inertial regressor, configuration covector, transverse dissipation, and nominal/Euclidean/Bregman modes. The Bregman mode uses an affine-invariant matrix exponential, so every finite estimate stays SPD.

`agc.batch` and `agc.opt` only evaluate independent scenarios. `agc.viz.replay3D` renders saved logs after simulation; optimization and batch jobs do not render.

`agc.viz.paperFigures` is likewise post-processing only. It reads one saved six-variant theory suite and exports static C1/C2 and Euclidean/Bregman comparison figures.
