# Architecture

`agc.sim.runScenario` is the only closed-loop execution path. A scenario carries true plant parameters, initial state and estimate, replay sampler, controller gains, and three fixed rates. The runner sends held body wrenches to a Robotics System Toolbox floating-body plant and logs paper-facing diagnostics.

`agc.paper` is intentionally function-based. It exposes the two Coriolis actions, inertial regressor, configuration covector, transverse dissipation, and nominal/Euclidean/Bregman modes. The Bregman mode uses an affine-invariant matrix exponential, so every finite estimate stays SPD.

`agc.batch` and `agc.opt` only evaluate independent scenarios. `agc.viz.replay3D` renders saved logs after simulation; optimization and batch jobs do not render.
