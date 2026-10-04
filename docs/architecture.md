# Architecture

`vgc.sim.default_scenario` builds a nominal scenario from a processed trajectory. `vgc.sim.run_scenario` advances the closed loop at the plant rate and updates the wrench at the controller rate. `vgc.paper.controller` computes the known-inertia geometric tracking wrench, while `vgc.plant.pybullet_plant` integrates the bare vehicle in PyBullet.

Visualization is isolated under `vgc.viz`; persistence and result discovery are provided by `vgc.io.persistence`. The runtime follows one fixed known-inertia control path.
