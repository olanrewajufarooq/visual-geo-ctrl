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

From the repository root on Windows:

```bat
.\trajectories\download_ratm_trajectories.cmd
```

To fetch only the autonomous set:

```bat
.\trajectories\download_ratm_trajectories.cmd autonomous
```

To fetch only the piloted set:

```bat
.\trajectories\download_ratm_trajectories.cmd piloted
```

If an interrupted run leaves partial files behind, retry with explicit cleanup:

```bat
.\trajectories\download_ratm_trajectories.cmd autonomous force-clean
```

The downloader always writes CSVs into `.\trajectories\autonomous` and `.\trajectories\piloted`, and keeps downloaded chunks plus rebuilt ZIPs in `.\trajectories\archive`.

The default second argument is `safe`, which reuses cached downloads but refuses to overwrite non-empty target folders.

## Papers

If you use these trajectories, cite the dataset repository and the papers it references:

1. Michael Bosello, Davide Aguiari, Yvo Keuter, Enrico Pallotta, Sara Kiade, Gyordan Caminati, Flavio Pinzarrone, Junaid Halepota, Jacopo Panerati, and Giovanni Pau. "Race Against the Machine: A Fully-Annotated, Open-Design Dataset of Autonomous and Piloted High-Speed Flight." IEEE Robotics and Automation Letters, 9(4):3799-3806, 2024. DOI: `10.1109/LRA.2024.3371288`
2. Michael Bosello, Flavio Pinzarrone, Sara Kiade, Davide Aguiari, Yvo Keuter, Aaesha AlShehhi, Gyordan Caminati, Kei Long Wong, Ka Seng Chou, Junaid Halepota, Fares Alneyadi, Jacopo Panerati, and Giovanni Pau. "On Your Own: Pro-Level Autonomous Drone Racing in Uninstrumented Arenas." IEEE Robotics and Automation Letters, 11(3):2674-2681, 2026. DOI: `10.1109/LRA.2026.3653405`

## Geometric Replay Findings

The downloaded files come from the TII Racing `drone-racing-dataset`. The
`*_500hz_freq_sync.csv` files are the appropriate source for simulation because
they are uniformly sampled at 500 Hz. The dataset's own preprocessing reports
that:

- `drone_[x/y/z]` is MoCap position in meters in the world frame.
- `drone_rot[0-8]` is a 3x3 MoCap rotation stored in column-major order. The
  rotation maps body-frame coordinates to world-frame coordinates.
- `drone_velocity_linear_[x/y/z]` is obtained by differentiating world position,
  so it is a world-frame velocity of the body origin.
- `drone_velocity_angular_[x/y/z]` is obtained from `dR/dt * R'`, so it is a
  world-frame angular velocity.
- The dataset interpolates rotations with spherical linear interpolation, not
  component-wise linear interpolation.

These conventions are consistent with the dataset README, its interpolation
script, and the companion paper (Bosello et al., IEEE RA-L 2024,
DOI: 10.1109/LRA.2024.3371288).

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

Differentiating the converted body signals is important. It includes the frame
transport terms caused by the changing attitude. In particular,

```text
d(R' v_world)/dt = R' * (dv_world/dt - omega_world x v_world)
```

The replay processor now performs this conversion before deriving acceleration.
The replay trajectory also interpolates orientation along `SO(3)` using the
relative rotation logarithm/Rodrigues formula rather than selecting the nearest
sample.

The pose remains in world coordinates as `H = [R p; 0 1]`, while the returned
twist and acceleration are body-coordinate quantities required by the existing
geometric controller. The input pose, velocity, and acceleration are therefore
not treated as six unrelated Euclidean values.

IMU `accel_[x/y/z]` is not substituted for `A`; it has sensor-frame and
specific-force/gravity conventions that require separate calibration.

Sources: [dataset repository](https://github.com/tii-racing/drone-racing-dataset),
[dataset interpolation script](https://raw.githubusercontent.com/tii-racing/drone-racing-dataset/main/scripts/data_interpolation.py),
and [companion paper](https://doi.org/10.1109/LRA.2024.3371288).
