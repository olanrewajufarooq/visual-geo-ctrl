# Adaptive Geo Control

MATLAB framework for simulating and analyzing fully actuated hexacopter dynamics on SE(3), with geometric controllers, online inertial adaptation, batch experiments, and result-generation utilities for research workflows.

## Quick Start

```matlab
startup
run_adaptive_reproduce
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

## Documentation

Full architecture, configuration, and workflow details live under [docs/](docs/README.md).

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
