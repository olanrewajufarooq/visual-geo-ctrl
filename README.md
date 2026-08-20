# Adaptive Geo Control

MATLAB framework for simulating and analyzing fully actuated hexacopter dynamics on SE(3), with geometric controllers, online inertial adaptation, batch experiments, and result-generation utilities for research workflows.

## Quick Start

```matlab
startup
run_nominal_demo
run_adaptive_demo
run_adaptive_gain_comparison
```

## Installation

```bash
git clone https://github.com/kfupm-arm-lab/adaptive-geo-ctrl.git
cd adaptive-geo-ctrl
```

```matlab
cd('path/to/adaptive-geo-ctrl')
startup
```

The root `startup.m` adds the repository root plus `src/` and `run/` to the active MATLAB path for the current session.

For automated or headless reproduction runs, use the dedicated batch entry point in `run/ci_release.m`.

## Feature Highlights

- SE(3) rigid-body hexacopter dynamics with optional ground-contact handling
- Multiple built-in trajectories including hover, circle, infinity, helix, and Lissajous paths
- PD, feedforward, and feedback-linearized wrench control workflows
- Euclidean and Bregman adaptive estimation for payload and inertia variation
- Batch runs across trajectories, gains, and controller configurations
- Live plotting, URDF-backed visualization, and saved summary figures
- CI-oriented release script and GitHub Actions automation

## Documentation

Detailed architecture, configuration, workflow, and extension guidance now lives under `docs/`.

- [Documentation Hub](docs/README.md)
- [Getting Started](docs/getting-started.md)
- [Configuration](docs/configuration.md)
- [Batch Simulations](docs/batch-simulations.md)
- [Features](docs/features.md)
- [Project Structure](docs/project-structure.md)
- [Simulation Outputs](docs/simulation-outputs.md)
- [CI/CD](docs/cicd.md)
- [Customization](docs/customization.md)
- [Troubleshooting](docs/troubleshooting.md)
- [Coding Conventions](docs/coding-conventions.md)

## Citations

If you use this framework in research, cite:

```bibtex
@misc{olanrewaju_geometrically_consistent_adaptive_control,
  title  = {Geometrically-Consistent Adaptive Control on SE(3) for Fully-Actuated Aerial Vehicles},
  author = {Farooq Olanrewaju, Sami El-Ferik, Muhammed Emzir, Ramy Rashad},
  note   = {Please replace this entry with the final publication details (venue/year/DOI) once available.}
}
```

## License

Released under the MIT License. See [LICENSE](LICENSE).

## Contributing

Contributions are welcome. Start from [docs/getting-started.md](docs/getting-started.md), follow the [coding conventions](docs/coding-conventions.md), and open a pull request with focused changes.

## Support

- Issues: <https://github.com/kfupm-arm-lab/adaptive-geo-ctrl/issues>
- Discussions: <https://github.com/kfupm-arm-lab/adaptive-geo-ctrl/discussions>
- Email: <g202404900@kfupm.edu.sa>, <olanrewajufarooq@yahoo.com>
