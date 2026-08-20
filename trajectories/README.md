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
