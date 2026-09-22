# Drone Racing Trajectories

This folder stores only trajectory CSV files extracted from the public `tii-racing/drone-racing-dataset` release assets.

## Source

- GitHub repository: <https://github.com/tii-racing/drone-racing-dataset>
- Release used by the downloader: `v3.0.0`

## Contents

- `autonomous/<flight-name>/flight-..._cam_ts_sync.csv`
- `autonomous/<flight-name>/flight-..._500hz_freq_sync.csv`
- `piloted/<flight-name>/flight-..._cam_ts_sync.csv`
- `piloted/<flight-name>/flight-..._500hz_freq_sync.csv`
- `archive/<group>\*_zipchunk*` and `archive/<group>.zip` as a local download cache

No images, labels, ROS bags, or quadrotor assets are stored here.

## Download

Cross-platform download using Python:

```powershell
python trajectories/download_ratm_trajectories.py
```

To fetch only the autonomous set:

```powershell
python trajectories/download_ratm_trajectories.py autonomous
```

To fetch only the piloted set:

```powershell
python trajectories/download_ratm_trajectories.py piloted
```

If an interrupted run leaves partial files behind, retry with explicit cleanup:

```powershell
python trajectories/download_ratm_trajectories.py autonomous --force-clean
```

The downloader always writes CSVs into `trajectories/autonomous` and `trajectories/piloted`, and keeps downloaded chunks plus rebuilt ZIPs in `trajectories/archive`.

The default behavior reuses cached downloads and protects non-empty target folders unless `--force-clean` is passed.

## Trajectory Datasets & Preprocessing

Canonical smoothed trajectory artifacts are stored in `trajectories/processed/*.npz` as compressed NumPy archives. These files contain continuous 500 Hz reference trajectories ($t, p, v, a, R, \Omega, \dot{\Omega}$) and metadata, consumed directly by `agc.sim.ReplayTrajectory`.

### Preprocessing Python Workflow

Raw 500 Hz CSV telemetry files are processed into canonical `.npz` reference artifacts using `trajectories/process_trajectories.py` and the `trajectories.replay_scripts` package:

```powershell
# Preprocess the held-out lemniscate and three optimization lemniscates
python trajectories/process_trajectories.py

# Use the legacy high-resolution WNOJ fit (much slower)
python trajectories/process_trajectories.py --full-precision

# Force re-processing from raw CSV flight recordings
python trajectories/process_trajectories.py --clear-cache

# Process all trajectories listed in manifest.json
python trajectories/process_trajectories.py --all
```

To inspect any recorded flight trajectory:
```powershell
python run/plot_trajectories.py --replay-id lemniscate_01_auto
```


## Papers

If you use these trajectories, cite the dataset repository and the papers it references:

1. Michael Bosello, Davide Aguiari, Yvo Keuter, Enrico Pallotta, Sara Kiade, Gyordan Caminati, Flavio Pinzarrone, Junaid Halepota, Jacopo Panerati, and Giovanni Pau. "Race Against the Machine: A Fully-Annotated, Open-Design Dataset of Autonomous and Piloted High-Speed Flight." IEEE Robotics and Automation Letters, 9(4):3799-3806, 2024. DOI: `10.1109/LRA.2024.3371288`
2. Michael Bosello, Flavio Pinzarrone, Sara Kiade, Davide Aguiari, Yvo Keuter, Aaesha AlShehhi, Gyordan Caminati, Kei Long Wong, Ka Seng Chou, Junaid Halepota, Fares Alneyadi, Jacopo Panerati, and Giovanni Pau. "On Your Own: Pro-Level Autonomous Drone Racing in Uninstrumented Arenas." IEEE Robotics and Automation Letters, 11(3):2674-2681, 2026. DOI: `10.1109/LRA.2026.3653405`

## Geometric Replay Findings

The downloaded files come from the TII Racing `drone-racing-dataset`. The `*_500hz_freq_sync.csv` files are the appropriate source for simulation because they are uniformly sampled at 500 Hz. The dataset's own preprocessing reports that:

- `drone_[x/y/z]` is MoCap position in meters in the world frame.
- `drone_rot[0-8]` is a 3x3 MoCap rotation stored in column-major order. The rotation maps body-frame coordinates to world-frame coordinates.
- `drone_velocity_linear_[x/y/z]` is obtained by differentiating world position, so it is a world-frame velocity of the body origin.
- `drone_velocity_angular_[x/y/z]` is obtained from `dR/dt * R'`, so it is a world-frame angular velocity.
- The dataset interpolates rotations with spherical linear interpolation, not component-wise linear interpolation.

These conventions are consistent with the dataset README, its interpolation script, and the companion paper (Bosello et al., IEEE RA-L 2024, DOI: 10.1109/LRA.2024.3371288).

`fth.traj.TrajectoryBase` and the controller expect

```text
V = [omega_body; v_body]
A = dV/dt in body coordinates
```

Therefore the CSV values cannot be passed directly as `Vd` and `Ad`:

```text
v_body     = R' * v_world
omega_body = R' * omega_world
a_body     = d(v_body)/dt
alpha_body = d(omega_body)/dt
```

Differentiating the converted body signals is important. It includes the frame transport terms caused by the changing attitude. In particular,

```text
d(R' v_world)/dt = R' * (dv_world/dt - omega_world x v_world)
```

The replay processor performs this conversion before fitting the joint pose,
twist, and acceleration trajectory on $SE(3)$.

### Acceleration Post-Processing

The dataset velocity channels are derived from MoCap pose data. Its `accel_*` IMU channels are sensor-frame specific force, not the kinematic acceleration required by `TrajectoryBase`, and must not be copied into `A`.

The controller uses world pose and body-coordinate twist and acceleration:

$$
H = \begin{bmatrix}R & p\\0 & 1\end{bmatrix},
\qquad
V = \begin{bmatrix}\omega_b\\v_b\end{bmatrix},
\qquad
A = \dot{V}.
$$

The processor converts the measured world-frame velocity channels to body
twist, then runs the nonlinear sparse $SE(3)$ WNOJ batch smoother using the
exact convention transformation $T=H^{-1}$, $\varpi=-V$, and
$\dot{\varpi}=-A$. Gaussian-process interpolation returns mutually consistent
$H$, $V$, and $A$ on the original sample grid. See
[POSTPROCESSING.md](POSTPROCESSING.md) for the equations, convergence settings,
and reference.

Sources: [dataset repository](https://github.com/tii-racing/drone-racing-dataset), [dataset interpolation script](https://raw.githubusercontent.com/tii-racing/drone-racing-dataset/main/scripts/data_interpolation.py), and [companion paper](https://doi.org/10.1109/LRA.2024.3371288).
