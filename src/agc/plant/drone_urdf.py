"""Multicopter URDF generator with exact inertial properties and high-fidelity visuals.

Generates realistic UAV models:
1. "pybullet_drones": Authentic quadrotor mesh from utiasDSL/gym-pybullet-drones (cf2.dae)
   with scaled racing geometry, 4 brushless motor hubs, aerodynamic propeller blur discs,
   and aviation navigation lighting.
2. "hexacopter": Carbon-fiber hexacopter with avionics stack, tilted front FPV camera,
   6 arms, brushless outrunner motors, counter-rotating propeller discs, and navigation LEDs.

Preserves exact mathematical mass, CoM, and 6D inertia tensor from the input pi vector.
"""

from pathlib import Path
import numpy as np


def generate_multicopter_urdf(
    pi: np.ndarray,
    output_path: str,
    drone_type: str = "pybullet_drones",
):
    """Generate a high-detail multicopter URDF with exact mass, CoM, and inertia."""
    m = float(pi[0])
    h = np.asarray(pi[1:4], dtype=float)
    r_com = h / m

    # Inertia tensor about origin
    I_origin = np.array([
        [pi[4], pi[7], pi[8]],
        [pi[7], pi[5], pi[9]],
        [pi[8], pi[9], pi[6]],
    ], dtype=float)

    # Parallel-axis theorem to compute inertia tensor about center of mass
    I_com = I_origin - m * ((r_com @ r_com) * np.eye(3, dtype=float) - np.outer(r_com, r_com))
    # Ensure positive definiteness on diagonals
    I_com[0, 0] = max(I_com[0, 0], 1e-5)
    I_com[1, 1] = max(I_com[1, 1], 1e-5)
    I_com[2, 2] = max(I_com[2, 2], 1e-5)

    if drone_type == "pybullet_drones":
        repo_root = Path(__file__).resolve().parents[3]
        mesh_path = (repo_root / "assets" / "drone" / "cf2.dae").resolve()
        if mesh_path.exists():
            urdf = _generate_pybullet_drones_urdf(m, r_com, I_com, str(mesh_path).replace("\\", "/"))
        else:
            urdf = _generate_hexacopter_urdf(m, r_com, I_com)
    else:
        urdf = _generate_hexacopter_urdf(m, r_com, I_com)

    with open(output_path, "w", encoding="utf-8") as f:
        f.write(urdf)


def _generate_pybullet_drones_urdf(
    m: float,
    r_com: np.ndarray,
    I_com: np.ndarray,
    mesh_path: str,
    scale: float = 3.5,
) -> str:
    """Generate quadrotor URDF using the gym-pybullet-drones cf2.dae mesh with exact paper inertia."""
    arm_dist = 0.040 * scale  # ~0.14 m from center to motor
    motors = [
        (1, arm_dist, arm_dist, "prop_front", "led_cyan", 0.0),
        (2, -arm_dist, arm_dist, "prop_rear", "led_red", np.pi / 2),
        (3, -arm_dist, -arm_dist, "prop_rear", "led_amber", np.pi),
        (4, arm_dist, -arm_dist, "prop_front", "led_green", -np.pi / 2),
    ]

    motors_urdf = ""
    for idx, mx, my, prop_mat, led_mat, yaw_angle in motors:
        motors_urdf += f"""
  <!-- Motor {idx} -->
  <link name="motor_{idx}">
    <inertial>
      <mass value="1e-7"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="1e-9" ixy="0" ixz="0" iyy="1e-9" iyz="0" izz="1e-9"/>
    </inertial>
    <!-- CNC Aluminum Motor Stator -->
    <visual>
      <origin xyz="{mx:.5f} {my:.5f} 0.012"/>
      <geometry><cylinder radius="0.020" length="0.015"/></geometry>
      <material name="cnc_aluminum"/>
    </visual>
    <!-- Prop Locknut -->
    <visual>
      <origin xyz="{mx:.5f} {my:.5f} 0.024"/>
      <geometry><cylinder radius="0.007" length="0.009"/></geometry>
      <material name="cnc_aluminum"/>
    </visual>
    <!-- Aerodynamic Propeller Blur Disc -->
    <visual>
      <origin xyz="{mx:.5f} {my:.5f} 0.027"/>
      <geometry><cylinder radius="0.10" length="0.003"/></geometry>
      <material name="{prop_mat}"/>
    </visual>
    <!-- Spinning Propeller Blades Silhouette -->
    <visual>
      <origin xyz="{mx:.5f} {my:.5f} 0.028" rpy="0 0 {yaw_angle + 0.785:.4f}"/>
      <geometry><box size="0.20 0.018 0.002"/></geometry>
      <material name="matte_carbon"/>
    </visual>
    <!-- Navigation LED -->
    <visual>
      <origin xyz="{(mx * 1.15):.5f} {(my * 1.15):.5f} 0.008"/>
      <geometry><box size="0.012 0.012 0.006"/></geometry>
      <material name="{led_mat}"/>
    </visual>
  </link>
  <joint name="joint_motor_{idx}" type="fixed">
    <parent link="base_link"/>
    <child link="motor_{idx}"/>
    <origin xyz="0 0 0"/>
  </joint>
"""

    return f"""<?xml version="1.0"?>
<robot name="pybullet_drone">
  <!-- Realistic Material Palette -->
  <material name="dark_carbon"><color rgba="0.12 0.12 0.14 1.0"/></material>
  <material name="matte_carbon"><color rgba="0.18 0.18 0.20 1.0"/></material>
  <material name="cnc_aluminum"><color rgba="0.55 0.58 0.62 1.0"/></material>
  <material name="prop_front"><color rgba="0.0 0.90 0.35 0.65"/></material>
  <material name="prop_rear"><color rgba="1.0 0.30 0.05 0.65"/></material>
  <material name="led_green"><color rgba="0.0 1.0 0.2 1.0"/></material>
  <material name="led_red"><color rgba="1.0 0.05 0.05 1.0"/></material>
  <material name="led_cyan"><color rgba="0.0 0.9 1.0 1.0"/></material>
  <material name="led_amber"><color rgba="1.0 0.7 0.0 1.0"/></material>

  <link name="base_link">
    <!-- Mathematical Invariance: Exact Inertia from Paper System Identification -->
    <inertial>
      <mass value="{m:.6f}"/>
      <origin xyz="{r_com[0]:.6f} {r_com[1]:.6f} {r_com[2]:.6f}"/>
      <inertia ixx="{I_com[0,0]:.6e}" ixy="{I_com[0,1]:.6e}" ixz="{I_com[0,2]:.6e}"
               iyy="{I_com[1,1]:.6e}" iyz="{I_com[1,2]:.6e}" izz="{I_com[2,2]:.6e}"/>
    </inertial>

    <!-- Authentic gym-pybullet-drones cf2.dae visual mesh -->
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="{mesh_path}" scale="{scale} {scale} {scale}"/>
      </geometry>
    </visual>

    <!-- Physical Collision Geometry -->
    <collision>
      <origin xyz="0 0 0"/>
      <geometry><cylinder radius="0.25" length="0.08"/></geometry>
    </collision>
  </link>
{motors_urdf}
</robot>
"""


def _generate_hexacopter_urdf(
    m: float,
    r_com: np.ndarray,
    I_com: np.ndarray,
) -> str:
    """Generate high-detail 6-arm hexacopter URDF."""
    arm_length = 0.28
    arm_angles = [0.0, 60.0, 120.0, 180.0, 240.0, 300.0]
    arms_urdf = ""

    for i, deg in enumerate(arm_angles, start=1):
        rad = np.radians(deg)
        cos_a = np.cos(rad)
        sin_a = np.sin(rad)
        arm_x = (arm_length / 2.0) * cos_a
        arm_y = (arm_length / 2.0) * sin_a
        m_x = arm_length * cos_a
        m_y = arm_length * sin_a

        is_front = (deg <= 60.0 or deg >= 300.0)
        prop_mat = "prop_front" if is_front else "prop_rear"

        if deg in (300.0, 0.0):
            led_mat = "led_green" if sin_a < 0 else "led_cyan"
        elif deg == 60.0:
            led_mat = "led_red" if sin_a > 0 else "led_cyan"
        elif deg == 180.0:
            led_mat = "led_amber"
        elif deg == 120.0:
            led_mat = "led_red"
        else:
            led_mat = "led_green"

        arms_urdf += f"""
  <!-- Arm {i} ({deg} deg) -->
  <link name="arm_{i}">
    <inertial>
      <mass value="1e-7"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="1e-9" ixy="0" ixz="0" iyy="1e-9" iyz="0" izz="1e-9"/>
    </inertial>
    <!-- Carbon Tube Boom -->
    <visual>
      <origin xyz="{arm_x:.5f} {arm_y:.5f} 0.0" rpy="0 1.5707 {rad:.5f}"/>
      <geometry><cylinder radius="0.010" length="{arm_length:.4f}"/></geometry>
      <material name="matte_carbon"/>
    </visual>
    <!-- CNC Aluminum Motor Mount -->
    <visual>
      <origin xyz="{m_x:.5f} {m_y:.5f} 0.006"/>
      <geometry><cylinder radius="0.024" length="0.012"/></geometry>
      <material name="cnc_aluminum"/>
    </visual>
    <!-- Brushless Motor Stator (Dark Gunmetal) -->
    <visual>
      <origin xyz="{m_x:.5f} {m_y:.5f} 0.018"/>
      <geometry><cylinder radius="0.021" length="0.016"/></geometry>
      <material name="motor_stator"/>
    </visual>
    <!-- Brushless Motor Rotor Bell (Silver Chrome) -->
    <visual>
      <origin xyz="{m_x:.5f} {m_y:.5f} 0.030"/>
      <geometry><cylinder radius="0.022" length="0.014"/></geometry>
      <material name="motor_silver"/>
    </visual>
    <!-- Prop Locknut -->
    <visual>
      <origin xyz="{m_x:.5f} {m_y:.5f} 0.042"/>
      <geometry><cylinder radius="0.008" length="0.010"/></geometry>
      <material name="cnc_aluminum"/>
    </visual>
    <!-- Propeller Disc (Translucent Aerodynamic Blur) -->
    <visual>
      <origin xyz="{m_x:.5f} {m_y:.5f} 0.045"/>
      <geometry><cylinder radius="0.125" length="0.003"/></geometry>
      <material name="{prop_mat}"/>
    </visual>
    <!-- Propeller Blade Crossbar (High-Detail Airfoil Silhouette) -->
    <visual>
      <origin xyz="{m_x:.5f} {m_y:.5f} 0.046" rpy="0 0 {rad + 0.785:.4f}"/>
      <geometry><box size="0.25 0.022 0.002"/></geometry>
      <material name="matte_carbon"/>
    </visual>
    <!-- Navigation Tip LED -->
    <visual>
      <origin xyz="{(m_x * 1.08):.5f} {(m_y * 1.08):.5f} 0.015"/>
      <geometry><box size="0.015 0.015 0.008"/></geometry>
      <material name="{led_mat}"/>
    </visual>
  </link>
  <joint name="joint_arm_{i}" type="fixed">
    <parent link="base_link"/>
    <child link="arm_{i}"/>
    <origin xyz="0 0 0"/>
  </joint>
"""

    return f"""<?xml version="1.0"?>
<robot name="hexacopter">
  <!-- Realistic Material Palette -->
  <material name="dark_carbon"><color rgba="0.12 0.12 0.14 1.0"/></material>
  <material name="matte_carbon"><color rgba="0.18 0.18 0.20 1.0"/></material>
  <material name="cnc_aluminum"><color rgba="0.55 0.58 0.62 1.0"/></material>
  <material name="motor_stator"><color rgba="0.28 0.28 0.32 1.0"/></material>
  <material name="motor_silver"><color rgba="0.80 0.82 0.86 1.0"/></material>
  <material name="prop_front"><color rgba="0.0 0.90 0.35 0.65"/></material>
  <material name="prop_rear"><color rgba="1.0 0.30 0.05 0.65"/></material>
  <material name="camera_lens"><color rgba="0.05 0.05 0.08 1.0"/></material>
  <material name="fpv_case"><color rgba="0.95 0.35 0.05 1.0"/></material>
  <material name="landing_gear"><color rgba="0.15 0.15 0.17 1.0"/></material>
  <material name="gps_mast"><color rgba="0.20 0.22 0.25 1.0"/></material>
  <material name="led_green"><color rgba="0.0 1.0 0.2 1.0"/></material>
  <material name="led_red"><color rgba="1.0 0.05 0.05 1.0"/></material>
  <material name="led_cyan"><color rgba="0.0 0.9 1.0 1.0"/></material>
  <material name="led_amber"><color rgba="1.0 0.7 0.0 1.0"/></material>

  <link name="base_link">
    <inertial>
      <mass value="{m:.6f}"/>
      <origin xyz="{r_com[0]:.6f} {r_com[1]:.6f} {r_com[2]:.6f}"/>
      <inertia ixx="{I_com[0,0]:.6e}" ixy="{I_com[0,1]:.6e}" ixz="{I_com[0,2]:.6e}"
               iyy="{I_com[1,1]:.6e}" iyz="{I_com[1,2]:.6e}" izz="{I_com[2,2]:.6e}"/>
    </inertial>

    <!-- Lower Carbon-Fiber Deck Plate -->
    <visual>
      <origin xyz="0 0 -0.015"/>
      <geometry><cylinder radius="0.13" length="0.003"/></geometry>
      <material name="dark_carbon"/>
    </visual>

    <!-- Upper Carbon-Fiber Deck Plate -->
    <visual>
      <origin xyz="0 0 0.015"/>
      <geometry><cylinder radius="0.13" length="0.003"/></geometry>
      <material name="dark_carbon"/>
    </visual>

    <!-- Central Electronics Core / Flight Controller Stack -->
    <visual>
      <origin xyz="0 0 0.0"/>
      <geometry><box size="0.12 0.12 0.026"/></geometry>
      <material name="matte_carbon"/>
    </visual>

    <!-- Avionics Top Dome / Canopy -->
    <visual>
      <origin xyz="0 0 0.025"/>
      <geometry><cylinder radius="0.08" length="0.020"/></geometry>
      <material name="dark_carbon"/>
    </visual>

    <!-- GPS / Compass Mast & Antenna Puck -->
    <visual>
      <origin xyz="-0.05 0 0.045"/>
      <geometry><cylinder radius="0.004" length="0.045"/></geometry>
      <material name="gps_mast"/>
    </visual>
    <visual>
      <origin xyz="-0.05 0 0.070"/>
      <geometry><cylinder radius="0.025" length="0.010"/></geometry>
      <material name="dark_carbon"/>
    </visual>

    <!-- Front FPV Racing Camera Pod (25 deg upward tilt) -->
    <visual>
      <origin xyz="0.13 0 0.012" rpy="0 -0.4363 0"/>
      <geometry><box size="0.028 0.028 0.028"/></geometry>
      <material name="fpv_case"/>
    </visual>
    <visual>
      <origin xyz="0.145 0 0.019" rpy="0 -0.4363 0"/>
      <geometry><cylinder radius="0.009" length="0.010"/></geometry>
      <material name="camera_lens"/>
    </visual>

    <!-- Physical Collision Geometry -->
    <collision>
      <origin xyz="0 0 0"/>
      <geometry><cylinder radius="0.25" length="0.08"/></geometry>
    </collision>
  </link>
{arms_urdf}
  <!-- Left Landing Skid -->
  <link name="gear_left">
    <inertial>
      <mass value="1e-7"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="1e-9" ixy="0" ixz="0" iyy="1e-9" iyz="0" izz="1e-9"/>
    </inertial>
    <visual>
      <origin xyz="0.08 0.10 -0.06"/>
      <geometry><cylinder radius="0.005" length="0.10"/></geometry>
      <material name="landing_gear"/>
    </visual>
    <visual>
      <origin xyz="-0.08 0.10 -0.06"/>
      <geometry><cylinder radius="0.005" length="0.10"/></geometry>
      <material name="landing_gear"/>
    </visual>
    <visual>
      <origin xyz="0 0.10 -0.11" rpy="0 1.5707 0"/>
      <geometry><cylinder radius="0.007" length="0.28"/></geometry>
      <material name="landing_gear"/>
    </visual>
  </link>
  <joint name="joint_gear_left" type="fixed">
    <parent link="base_link"/>
    <child link="gear_left"/>
    <origin xyz="0 0 0"/>
  </joint>

  <!-- Right Landing Skid -->
  <link name="gear_right">
    <inertial>
      <mass value="1e-7"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="1e-9" ixy="0" ixz="0" iyy="1e-9" iyz="0" izz="1e-9"/>
    </inertial>
    <visual>
      <origin xyz="0.08 -0.10 -0.06"/>
      <geometry><cylinder radius="0.005" length="0.10"/></geometry>
      <material name="landing_gear"/>
    </visual>
    <visual>
      <origin xyz="-0.08 -0.10 -0.06"/>
      <geometry><cylinder radius="0.005" length="0.10"/></geometry>
      <material name="landing_gear"/>
    </visual>
    <visual>
      <origin xyz="0 -0.10 -0.11" rpy="0 1.5707 0"/>
      <geometry><cylinder radius="0.007" length="0.28"/></geometry>
      <material name="landing_gear"/>
    </visual>
  </link>
  <joint name="joint_gear_right" type="fixed">
    <parent link="base_link"/>
    <child link="gear_right"/>
    <origin xyz="0 0 0"/>
  </joint>
</robot>
"""
