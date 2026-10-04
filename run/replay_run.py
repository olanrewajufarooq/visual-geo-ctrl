"""Replay a saved simulation run in PyBullet 3D GUI with advanced visualization."""

import sys
import time
from pathlib import Path
import numpy as np
import pybullet as p
import pybullet_data

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "src"))

from vgc.io.persistence import load_run
from vgc.math.se3 import rotm_to_quat
from vgc.plant.suppress import suppress_c_stdout


from vgc.viz.arena_scene import ArenaScene
from vgc.viz.race_gates import RaceGateManager
from vgc.viz.fpv_osd import FpvOsd


def replay_3d(run_dir: str, frame_stride: int = 5, playback_speed: float = 1.0):
    """Replay saved run poses in interactive PyBullet GUI with full multicopter visuals."""
    run = load_run(run_dir)
    t = run["t"]
    H_actual = run["H"]
    H_desired = run["Hdesired"]
    n = len(t)
    dt_step = (t[frame_stride] - t[0]) if n > frame_stride else 0.01

    client_id = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 1)

    # Detect floor level: if trajectory reaches near 0, ground is at 0.0
    ground_z = 0.0 if np.min(H_actual[:, 2, 3]) >= -0.2 else -1.5
    with suppress_c_stdout():
        p.loadURDF("plane.urdf", [0, 0, ground_z])

    # Realistic flight arena scene
    arena = ArenaScene(client_id=client_id, ground_z=ground_z, style="arena")

    # Load racing gates
    gate_mgr = RaceGateManager(client_id=client_id)
    z_off = ground_z if ground_z != 0 else 0.0
    if np.max(H_desired[:, 2, 3]) > 3.0:
        gate_mgr.load_ratm_track(z_offset=z_off)
    else:
        gate_mgr.load_lemniscate_4gates(z_offset=z_off)

    # Load high-detail drone URDF (defaults to gym-pybullet-drones cf2)
    cf2_path = REPO_ROOT / "assets" / "drone" / "cf2.urdf"
    urdf_path = str(cf2_path if cf2_path.is_file() else (REPO_ROOT / "assets" / "hexacopter.urdf"))
    with suppress_c_stdout():
        uav = p.loadURDF(urdf_path, [0, 0, 0], [0, 0, 0, 1], flags=p.URDF_MERGE_FIXED_LINKS)

    # Attach local body triad (Red=X forward, Green=Y left, Blue=Z up)
    p.addUserDebugLine([0, 0, 0], [0.35, 0, 0], [1.0, 0.1, 0.1], 3.0, parentObjectUniqueId=uav)
    p.addUserDebugLine([0, 0, 0], [0, 0.35, 0], [0.1, 0.95, 0.2], 3.0, parentObjectUniqueId=uav)
    p.addUserDebugLine([0, 0, 0], [0, 0, 0.35], [0.15, 0.45, 1.0], 3.0, parentObjectUniqueId=uav)

    # Pre-render desired reference trajectory in sky-cyan
    des_pts = H_desired[:, 0:3, 3]
    for i in range(0, len(des_pts) - 10, 10):
        p.addUserDebugLine(des_pts[i].tolist(), des_pts[i + 10].tolist(), [0.1, 0.75, 1.0], lineWidth=2.5)

    fpv_osd = FpvOsd(client_id=client_id, enabled=True)

    print("=" * 60)
    print(f"Replaying {run_dir} in PyBullet 3D GUI at {playback_speed:.1f}x speed.")
    print("Use mouse left-click/drag to orbit, right-click/drag to pan, scroll to zoom.")
    print("Press Ctrl+C in terminal to stop.")
    print("=" * 60)

    cam_target = np.array(H_actual[0, 0:3, 3], dtype=float)
    prev_trail_pos = None
    drop_line_id = None
    hud_id = None

    t_wall_start = time.perf_counter()

    try:
        for k in range(0, n, frame_stride):
            curr_t = float(t[k])
            pos = H_actual[k, 0:3, 3]
            R = H_actual[k, 0:3, 0:3]
            quat = rotm_to_quat(R).tolist()
            pos_list = pos.tolist()

            p.resetBasePositionAndOrientation(uav, pos_list, quat)

            # Smooth camera tracking (preserves user mouse orbit/zoom)
            alpha = 0.08
            cam_target = (1.0 - alpha) * cam_target + alpha * pos
            cam_info = p.getDebugVisualizerCamera()
            if cam_info is not None and len(cam_info) >= 11:
                u_yaw, u_pitch, u_dist = cam_info[8], cam_info[9], cam_info[10]
            else:
                u_yaw, u_pitch, u_dist = 45.0, -25.0, 2.8

            p.resetDebugVisualizerCamera(
                cameraDistance=u_dist,
                cameraYaw=u_yaw,
                cameraPitch=u_pitch,
                cameraTargetPosition=cam_target.tolist(),
            )

            # Check gate traversal
            gate_mgr.check_traversals(drone_pos=pos, prev_pos=prev_trail_pos, t=curr_t)

            # Flown trajectory trail (amber/gold, decimated)
            if prev_trail_pos is None:
                prev_trail_pos = pos
            elif np.linalg.norm(pos - prev_trail_pos) >= 0.03:
                p.addUserDebugLine(prev_trail_pos.tolist(), pos_list, [1.0, 0.78, 0.15], lineWidth=2.0)
                prev_trail_pos = pos

            # Ground drop shadow line
            drop_line_id = p.addUserDebugLine(
                pos_list,
                [pos[0], pos[1], ground_z],
                [0.3, 0.3, 0.3],
                lineWidth=1.0,
                replaceItemUniqueId=drop_line_id if drop_line_id is not None else -1,
            )

            # FPV OSD overlay
            if k % (frame_stride * 3) == 0:
                pos_err = float(np.linalg.norm(pos - H_desired[k, 0:3, 3]))
                vel_k = run["V"][k, 3:6] if "V" in run else np.zeros(3)
                fpv_osd.update(
                    t=curr_t,
                    pos=pos,
                    R=R,
                    vel=vel_k,
                    pos_err=pos_err,
                    s_norm=float(np.linalg.norm(run["s"][k])) if "s" in run else 0.0,
                    est_m=float(run["estimatePi"][k, 0]) if "estimatePi" in run else 3.65,
                    true_m=float(run["activePlantPi"][k, 0]) if "activePlantPi" in run else 3.65,
                    mode="NOMINAL",
                    coriolis=str(run.get("coriolis", "LC")).upper(),
                    sim_speed=playback_speed,
                )

            # Wall-clock pacing
            t_target = curr_t / playback_speed
            t_elapsed = time.perf_counter() - t_wall_start
            sleep_dt = t_target - t_elapsed
            if sleep_dt > 0.001:
                time.sleep(sleep_dt)

    except KeyboardInterrupt:
        pass
    finally:
        p.disconnect()


if __name__ == "__main__":
    import argparse
    from vgc.io.persistence import resolve_result_suite

    parser = argparse.ArgumentParser(description="Replay a saved simulation run in PyBullet 3D GUI.")
    parser.add_argument("run_dir", nargs="?", default=None, help="Path to saved run directory containing run.npz")
    parser.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier (e.g. 1.0, 2.0)")
    parser.add_argument("--frame-stride", type=int, default=5, help="Simulation step stride for visualization rendering")
    args = parser.parse_args()

    if args.run_dir is not None:
        target_dir = Path(args.run_dir)
        if not (target_dir / "run.npz").is_file():
            # Check if it's a suite directory
            candidates = [d for d in target_dir.iterdir() if d.is_dir() and (d / "run.npz").is_file()]
            if candidates:
                target_dir = candidates[0]
                print(f"Selected variant run: {target_dir.name}")
    else:
        # Default to latest result suite
        try:
            suite = resolve_result_suite()
            candidates = [d for d in suite.iterdir() if d.is_dir() and (d / "run.npz").is_file()]
            target_dir = candidates[0] if candidates else suite
        except FileNotFoundError:
            target_dir = REPO_ROOT / "results" / "inplace" / "nominal_lc"

    if not (target_dir / "run.npz").is_file():
        print(f"Error: No run.npz found in {target_dir}. Please run a simulation first.")
        sys.exit(1)

    replay_3d(str(target_dir), frame_stride=args.frame_stride, playback_speed=args.speed)
