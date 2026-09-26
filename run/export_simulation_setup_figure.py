"""Export publication-quality simulation-setup figures for IEEE papers.

Renders the complete authentic simulation scenario with all physical details:
1. Full 25m x 10m flight arena with 4 vertical truss columns, top cables, runway stripes, and launch pad.
2. 4 authentic racing gates along the lemniscate track.
3. Full 12.2m lemniscate reference trajectory passing through all 4 gates.
4. Fully actuated hexacopter at pre-release hover (t = 9.0s) with attached eccentric evaluation payload.
5. All materials transformed to realistic, publication-grade tones (brushed titanium columns,
   carbon-fiber gate frames, high-contrast banners, matte slate epoxy floor, zero specular glare).
6. Detail vehicle callout with delicate, highly transparent reference trajectory (alpha = 0.20).
7. IEEE-style two-panel composite figure with standard subfigure captions below each panel.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Dict, Any, List
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import pybullet as p
import pybullet_data
from scipy.interpolate import interp1d

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "src"))

from agc.sim.default_scenario import get_payload_profile


def render_scene(
    output_dir: Path,
    width: int = 2400,
    height: int = 1350,
    export_composite: bool = True,
) -> Dict[str, Path]:
    """Render and export realistic publication simulation figures."""
    output_dir.mkdir(parents=True, exist_ok=True)

    # 1. State at t = 9.0s (hover pre-release at central crossing)
    uav_pos = [0.10715, 0.14955, 0.672]
    uav_quat = [0.0, 0.0, 0.0, 1.0]

    # 2. Connect to PyBullet (GUI hardware OpenGL with fallback to DIRECT TinyRenderer)
    try:
        cid = p.connect(p.GUI, options=f"--width={width} --height={height}")
        renderer = p.ER_BULLET_HARDWARE_OPENGL
    except Exception:
        cid = p.connect(p.DIRECT)
        renderer = p.ER_TINY_RENDERER

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)

    generated_files: Dict[str, Path] = {}

    try:
        # 3. Outer studio ground plane: clean neutral matte studio grey
        floor_vis = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[60.0, 60.0, 0.05],
            rgbaColor=[0.90, 0.91, 0.93, 1.0],
            specularColor=[0.0, 0.0, 0.0],
            physicsClientId=cid,
        )
        p.createMultiBody(0, -1, floor_vis, [0, 0, -0.05], physicsClientId=cid)

        # 4. Load authentic arena scene URDF (enclosure, truss columns, cables, launch pad)
        arena_urdf = REPO_ROOT / "assets" / "arena" / "arena_scene.urdf"
        arena_id = p.loadURDF(
            str(arena_urdf),
            [0.0, 0.0, 0.0],
            useFixedBase=True,
            flags=p.URDF_MERGE_FIXED_LINKS,
            physicsClientId=cid,
        )

        # 5. Load authentic 4 race gates URDF
        gates_urdf = REPO_ROOT / "assets" / "gates" / "lemniscate_gates.urdf"
        gates_id = p.loadURDF(
            str(gates_urdf),
            [0.0, 0.0, 0.0],
            useFixedBase=True,
            flags=p.URDF_MERGE_FIXED_LINKS,
            physicsClientId=cid,
        )

        # 6. Transform arena materials to realistic, professional palette (no bright yellow!)
        for shape in p.getVisualShapeData(arena_id, physicsClientId=cid):
            link_idx = shape[1]
            rgba = list(shape[7])
            # Replace hazard yellow border with clean high-contrast safety border
            if rgba[0] > 0.8 and rgba[1] > 0.6 and rgba[2] < 0.2:
                p.changeVisualShape(
                    arena_id, link_idx, rgbaColor=[0.85, 0.88, 0.92, 1.0], specularColor=[0.0, 0.0, 0.0], physicsClientId=cid
                )
            # Floor epoxy: realistic polished industrial slate/epoxy floor, zero glare
            elif rgba[0] < 0.2 and rgba[1] < 0.2 and rgba[2] < 0.2:
                p.changeVisualShape(
                    arena_id, link_idx, rgbaColor=[0.32, 0.35, 0.39, 1.0], specularColor=[0.0, 0.0, 0.0], physicsClientId=cid
                )
            # Launch pad inner yellow ring -> clean metallic silver / white
            elif rgba[0] > 0.8 and rgba[1] > 0.8 and rgba[2] < 0.2:
                p.changeVisualShape(
                    arena_id, link_idx, rgbaColor=[0.85, 0.88, 0.92, 1.0], specularColor=[0.0, 0.0, 0.0], physicsClientId=cid
                )
            # Truss columns -> realistic brushed titanium / dark industrial steel
            elif abs(rgba[0] - 0.35) < 0.05 and abs(rgba[1] - 0.38) < 0.05:
                p.changeVisualShape(
                    arena_id, link_idx, rgbaColor=[0.25, 0.27, 0.30, 1.0], specularColor=[0.1, 0.1, 0.1], physicsClientId=cid
                )
            # Top perimeter cables -> realistic steel tension cable
            elif abs(rgba[0] - 0.25) < 0.05 and abs(rgba[1] - 0.4) < 0.05:
                p.changeVisualShape(
                    arena_id, link_idx, rgbaColor=[0.30, 0.32, 0.36, 1.0], specularColor=[0.0, 0.0, 0.0], physicsClientId=cid
                )

        # 7. Transform race gates to realistic professional appearance (no bright yellow!)
        for shape in p.getVisualShapeData(gates_id, physicsClientId=cid):
            link_idx = shape[1]
            rgba = list(shape[7])
            # Replace yellow/gold badge with clean white/silver badge
            if rgba[0] > 0.8 and rgba[1] > 0.7 and rgba[2] < 0.2:
                p.changeVisualShape(
                    gates_id, link_idx, rgbaColor=[0.88, 0.90, 0.94, 1.0], specularColor=[0.1, 0.1, 0.1], physicsClientId=cid
                )
            # Gate uprights & lintel crossbars -> matte carbon / dark steel
            elif abs(rgba[0] - 0.68) < 0.05 and abs(rgba[1] - 0.70) < 0.05:
                p.changeVisualShape(
                    gates_id, link_idx, rgbaColor=[0.20, 0.22, 0.25, 1.0], specularColor=[0.05, 0.05, 0.05], physicsClientId=cid
                )
            # Banners -> professional racing deep crimson / contrasting race banner
            elif rgba[0] > 0.8 and rgba[1] < 0.5:
                p.changeVisualShape(
                    gates_id, link_idx, rgbaColor=[0.85, 0.25, 0.12, 1.0], specularColor=[0.05, 0.05, 0.05], physicsClientId=cid
                )

        # 8. Load Hexacopter URDF
        urdf_file = str(REPO_ROOT / "assets" / "hexacopter.urdf")
        uav_id = p.loadURDF(urdf_file, uav_pos, uav_quat, physicsClientId=cid)

        # Semi-transparent muted rotor discs (slate-teal front, terracotta rear)
        for shape in p.getVisualShapeData(uav_id, physicsClientId=cid):
            link_idx = shape[1]
            rgba = list(shape[7])
            if rgba[1] > 0.7 and rgba[0] < 0.2:
                p.changeVisualShape(uav_id, link_idx, rgbaColor=[0.18, 0.46, 0.44, 0.45], physicsClientId=cid)
            elif rgba[0] > 0.8 and rgba[1] < 0.4:
                p.changeVisualShape(uav_id, link_idx, rgbaColor=[0.55, 0.26, 0.20, 0.45], physicsClientId=cid)

        # 9. Attach evaluation offset payload
        payload_def = get_payload_profile("evaluation")
        d_p = payload_def["dimensions"]
        center = payload_def["center"]
        payload_pos = np.array(uav_pos) + center

        vis_shape = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=(d_p / 2.0).tolist(),
            rgbaColor=[0.92, 0.46, 0.08, 1.0],
            specularColor=[0.2, 0.2, 0.2],
            physicsClientId=cid,
        )
        p.createMultiBody(
            baseMass=payload_def["mass"],
            baseVisualShapeIndex=vis_shape,
            basePosition=payload_pos.tolist(),
            baseOrientation=uav_quat,
            physicsClientId=cid,
        )

        # Rigid mounting strut connecting payload top to UAV chassis
        strut_top = np.array(uav_pos) + np.array([center[0], center[1], -0.015])
        strut_bot = np.array(uav_pos) + np.array([center[0], center[1], center[2] + d_p[2] / 2.0])
        strut_mid = (strut_top + strut_bot) / 2.0
        strut_len = float(np.linalg.norm(strut_top - strut_bot))
        strut_vis = p.createVisualShape(
            p.GEOM_CYLINDER,
            radius=0.005,
            length=strut_len,
            rgbaColor=[0.20, 0.22, 0.24, 1.0],
            physicsClientId=cid,
        )
        p.createMultiBody(0, -1, strut_vis, strut_mid.tolist(), uav_quat, physicsClientId=cid)

        # 10. Sample lemniscate reference trajectory (1 clean cycle threading through the 4 gates)
        traj_bodies: List[int] = []
        traj_npz = REPO_ROOT / "trajectories" / "processed" / "lemniscate_01_auto.npz"
        sampled_pts = []
        if traj_npz.is_file():
            data = np.load(traj_npz)
            pts = data["p"]
            t = data["t"]

            mask = (t >= 12.9) & (t <= 17.4)
            loop_pts = pts[mask]

            s = np.linspace(0, 1, len(loop_pts))
            s_fine = np.linspace(0, 1, 280)
            interp_fn = interp1d(s, loop_pts, axis=0, kind="cubic")
            sampled_pts = interp_fn(s_fine)
            sampled_pts[-1] = sampled_pts[0]

        def build_trajectory(rgba: List[float], radius: float):
            nonlocal traj_bodies
            for b in traj_bodies:
                p.removeBody(b, physicsClientId=cid)
            traj_bodies.clear()
            z_axis = np.array([0, 0, 1])
            for i in range(len(sampled_pts) - 1):
                p1 = sampled_pts[i]
                p2 = sampled_pts[i + 1]
                mid = (p1 + p2) / 2.0
                diff = p2 - p1
                length = float(np.linalg.norm(diff))
                if length > 1e-4:
                    cyl = p.createVisualShape(
                        p.GEOM_CYLINDER,
                        radius=radius,
                        length=length,
                        rgbaColor=rgba,
                        physicsClientId=cid,
                    )
                    dir_vec = diff / length
                    axis = np.cross(z_axis, dir_vec)
                    axis_len = float(np.linalg.norm(axis))
                    rot_q = [0, 0, 0, 1]
                    if axis_len > 1e-6:
                        angle = float(np.arccos(np.clip(np.dot(z_axis, dir_vec), -1.0, 1.0)))
                        rot_q = p.getQuaternionFromAxisAngle(axis / axis_len, angle)
                    bid = p.createMultiBody(0, -1, cyl, mid.tolist(), rot_q, physicsClientId=cid)
                    traj_bodies.append(bid)

        # Build standard trajectory for overview views (r=0.020, alpha=0.95)
        build_trajectory(rgba=[0.10, 0.65, 0.95, 0.95], radius=0.020)

        # 11. Camera Configurations
        overview_cameras = {
            "cand1_standard_3q": {
                "desc": "Candidate A (Primary): Exact Screenshot.png framing with all arena details & gates",
                "target": [0.35, -0.15, 1.20],
                "dist": 14.5,
                "yaw": 35.0,
                "pitch": -32.0,
                "fov": 48.0,
                "filename": "simulation_setup_cand1_standard_3q.png",
                "is_primary": True,
            },
            "cand2_top_elevated_3q": {
                "desc": "Candidate C: Elevated three-quarter view through all 4 gates",
                "target": [0.35, -0.15, 1.20],
                "dist": 13.5,
                "yaw": 38.0,
                "pitch": -38.0,
                "fov": 48.0,
                "filename": "simulation_setup_cand2_top_elevated_3q.png",
                "is_primary": False,
            },
            "cand3_alt_azimuth_3q": {
                "desc": "Candidate B: Tighter arena framing keeping all columns and gates in frame",
                "target": [0.35, -0.15, 1.00],
                "dist": 12.0,
                "yaw": 35.0,
                "pitch": -30.0,
                "fov": 45.0,
                "filename": "simulation_setup_cand3_alt_azimuth_3q.png",
                "alias": "simulation_setup_cand3_low_3q.png",
                "is_primary": False,
            },
        }

        rendered_images: Dict[str, Image.Image] = {}

        for key, cfg in overview_cameras.items():
            view_mat = p.computeViewMatrixFromYawPitchRoll(
                cameraTargetPosition=cfg["target"],
                distance=cfg["dist"],
                yaw=cfg["yaw"],
                pitch=cfg["pitch"],
                roll=0.0,
                upAxisIndex=2,
                physicsClientId=cid,
            )
            proj_mat = p.computeProjectionMatrixFOV(
                fov=cfg["fov"],
                aspect=float(width) / float(height),
                nearVal=0.1,
                farVal=60.0,
                physicsClientId=cid,
            )

            img_data = p.getCameraImage(
                width,
                height,
                viewMatrix=view_mat,
                projectionMatrix=proj_mat,
                shadow=1,
                lightDirection=[1.0, 1.0, 2.5],
                renderer=renderer,
                flags=p.ER_NO_SEGMENTATION_MASK,
                physicsClientId=cid,
            )

            rgb = np.reshape(img_data[2], (height, width, 4))
            pil_img = Image.fromarray(rgb)
            rendered_images[key] = pil_img

            out_file = output_dir / cfg["filename"]
            pil_img.save(out_file, "PNG", optimize=True)
            generated_files[key] = out_file
            print(f"[EXPORT] Saved {cfg['desc']} -> {out_file}")

            if cfg.get("is_primary"):
                prim_file = output_dir / "simulation_setup.png"
                pil_img.save(prim_file, "PNG", optimize=True)
                generated_files["primary"] = prim_file
                print(f"[EXPORT] Saved primary figure -> {prim_file}")

            if "alias" in cfg:
                alias_file = output_dir / cfg["alias"]
                pil_img.save(alias_file, "PNG", optimize=True)
                generated_files[cfg["alias"]] = alias_file

        # 12. Render Detail View with Delicate, Transparent Trajectory (alpha = 0.20, r = 0.008m)
        build_trajectory(rgba=[0.10, 0.65, 0.95, 0.20], radius=0.008)

        detail_cfg = {
            "desc": "High-resolution vehicle & eccentric payload detail callout",
            "target": [uav_pos[0] + 0.05, uav_pos[1] + 0.02, uav_pos[2] - 0.06],
            "dist": 1.45,
            "yaw": 52.0,
            "pitch": -26.0,
            "fov": 42.0,
            "filename": "simulation_setup_vehicle_detail.png",
        }

        view_mat_det = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=detail_cfg["target"],
            distance=detail_cfg["dist"],
            yaw=detail_cfg["yaw"],
            pitch=detail_cfg["pitch"],
            roll=0.0,
            upAxisIndex=2,
            physicsClientId=cid,
        )
        proj_mat_det = p.computeProjectionMatrixFOV(
            fov=detail_cfg["fov"],
            aspect=float(width) / float(height),
            nearVal=0.1,
            farVal=20.0,
            physicsClientId=cid,
        )

        img_data_det = p.getCameraImage(
            width,
            height,
            viewMatrix=view_mat_det,
            projectionMatrix=proj_mat_det,
            shadow=1,
            lightDirection=[1.0, 1.0, 2.5],
            renderer=renderer,
            flags=p.ER_NO_SEGMENTATION_MASK,
            physicsClientId=cid,
        )

        rgb_det = np.reshape(img_data_det[2], (height, width, 4))
        detail_img = Image.fromarray(rgb_det)
        rendered_images["vehicle_payload_callout"] = detail_img

        out_file_det = output_dir / detail_cfg["filename"]
        detail_img.save(out_file_det, "PNG", optimize=True)
        generated_files["vehicle_payload_callout"] = out_file_det
        print(f"[EXPORT] Saved {detail_cfg['desc']} -> {out_file_det}")

        # 13. Generate Composite Multi-Panel Publication Figure with IEEE-style Subfigure Captions Below Panels
        if export_composite and "cand1_standard_3q" in rendered_images:
            comp_file = output_dir / "simulation_setup_composite.png"
            create_composite_figure(
                overview_img=rendered_images["cand1_standard_3q"],
                detail_img=detail_img,
                out_path=comp_file,
            )
            generated_files["composite"] = comp_file
            print(f"[EXPORT] Saved IEEE composite publication figure -> {comp_file}")

    finally:
        p.disconnect(cid)

    return generated_files


def create_composite_figure(
    overview_img: Image.Image,
    detail_img: Image.Image,
    out_path: Path,
):
    """Create a publication-quality composite figure with subfigure captions below panels."""
    total_w = 2800
    panel_h = 1350
    caption_h = 110
    total_h = panel_h + caption_h

    w_left = 1750
    w_right = total_w - w_left - 16

    left_scaled = overview_img.resize((w_left, panel_h), Image.Resampling.LANCZOS)
    crop_w = int(detail_img.height * (w_right / panel_h))
    cx = detail_img.width // 2
    detail_cropped = detail_img.crop((cx - crop_w // 2, 0, cx + crop_w // 2, detail_img.height))
    right_scaled = detail_cropped.resize((w_right, panel_h), Image.Resampling.LANCZOS)

    canvas = Image.new("RGBA", (total_w, total_h), (255, 255, 255, 255))
    canvas.paste(left_scaled, (0, 0))
    canvas.paste(right_scaled, (w_left + 16, 0))

    draw = ImageDraw.Draw(canvas)
    try:
        font_reg = ImageFont.truetype("arial.ttf", 36)
    except Exception:
        font_reg = ImageFont.load_default()

    # IEEE Subfigure Caption (a) below left panel
    cap_a = "(a) Simulation arena and desired lemniscate trajectory."
    bbox_a = draw.textbbox((0, 0), cap_a, font=font_reg)
    text_w_a = bbox_a[2] - bbox_a[0]
    pos_x_a = (w_left - text_w_a) // 2
    draw.text((pos_x_a, panel_h + 35), cap_a, fill=(30, 35, 42, 255), font=font_reg)

    # IEEE Subfigure Caption (b) below right panel
    cap_b = "(b) Fully actuated hexacopter with attached eccentric payload."
    bbox_b = draw.textbbox((0, 0), cap_b, font=font_reg)
    text_w_b = bbox_b[2] - bbox_b[0]
    pos_x_b = (w_left + 16) + (w_right - text_w_b) // 2
    draw.text((pos_x_b, panel_h + 35), cap_b, fill=(30, 35, 42, 255), font=font_reg)

    canvas.save(out_path, "PNG", optimize=True)


def main():
    parser = argparse.ArgumentParser(description="Export IEEE simulation setup figure.")
    parser.add_argument("--output-dir", type=str, default="results/papers/sim_env")
    parser.add_argument("--width", type=int, default=2400)
    parser.add_argument("--height", type=int, default=1350)
    parser.add_argument("--no-composite", action="store_true", help="Skip composite figure generation")
    args = parser.parse_args()

    out_dir = Path(args.output_dir)
    if not out_dir.is_absolute():
        out_dir = REPO_ROOT / out_dir

    render_scene(
        output_dir=out_dir,
        width=args.width,
        height=args.height,
        export_composite=not args.no_composite,
    )


if __name__ == "__main__":
    main()
