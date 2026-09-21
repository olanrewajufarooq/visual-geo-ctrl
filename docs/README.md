# Documentation

- [Architecture](architecture.md): Overview of PyBullet multi-body simulation plant and paper-aligned geometric adaptive controller on $\mathrm{SE}(3)$.
- [PyBullet vs MATLAB Differences](pybullet_matlab_differences.md): Documented differences between PyBullet floating-base multi-body physics and MATLAB Robotics System Toolbox.
- [Getting Started](getting-started.md): Installation, Conda environment setup, running simulations, and automated tests.
- [Project Structure](project-structure.md): Layout of Python packages (`agc.math`, `agc.paper`, `agc.plant`, `agc.sim`, `agc.opt`, `agc.viz`, `agc.io`).
- [Batch Simulations](batch-simulations.md): Executing 6-variant comparative study suites in serial and parallel modes.
- [Gain Tuning](gain-tuning.md): Staged block-coordinate optimization schedule, objectives, and promotion rules.
- [Simulation Outputs](simulation-outputs.md): Data schema for `.npz` trajectory archives, `metadata.json`, and `manifest.json`.
- [Troubleshooting](troubleshooting.md): Common configuration issues, environment debugging, and FAQ.
- [Run Scripts](../run/README.md): Comprehensive CLI reference for all scripts under `run/`.
