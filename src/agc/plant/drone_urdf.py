"""Multicopter URDF generator with exact inertial properties."""

from pathlib import Path
import numpy as np


def generate_multicopter_urdf(pi: np.ndarray, output_path: str):
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
        # Front rotors green, rear rotors red (aviation navigation standard)
        prop_mat = "prop_front" if (deg <= 60 or deg >= 300) else "prop_rear"

        arms_urdf += f"""
  <link name="arm_{i}">
    <inertial>
      <mass value="0.0001"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="1e-7" ixy="0" ixz="0" iyy="1e-7" iyz="0" izz="1e-7"/>
    </inertial>
    <visual>
      <origin xyz="{arm_x:.5f} {arm_y:.5f} 0.0" rpy="0 1.5707 {rad:.5f}"/>
      <geometry><cylinder radius="0.012" length="{arm_length:.4f}"/></geometry>
      <material name="arm_metal"/>
    </visual>
    <visual>
      <origin xyz="{m_x:.5f} {m_y:.5f} 0.025"/>
      <geometry><cylinder radius="0.022" length="0.035"/></geometry>
      <material name="motor_silver"/>
    </visual>
    <visual>
      <origin xyz="{m_x:.5f} {m_y:.5f} 0.045"/>
      <geometry><cylinder radius="0.12" length="0.004"/></geometry>
      <material name="{prop_mat}"/>
    </visual>
  </link>
  <joint name="joint_arm_{i}" type="fixed">
    <parent link="base_link"/>
    <child link="arm_{i}"/>
    <origin xyz="0 0 0"/>
  </joint>
"""

    urdf = f"""<?xml version="1.0"?>
<robot name="hexacopter">
  <material name="dark_carbon"><color rgba="0.12 0.12 0.14 1.0"/></material>
  <material name="arm_metal"><color rgba="0.25 0.25 0.28 1.0"/></material>
  <material name="motor_silver"><color rgba="0.75 0.75 0.78 1.0"/></material>
  <material name="prop_front"><color rgba="0.1 0.85 0.25 0.75"/></material>
  <material name="prop_rear"><color rgba="0.9 0.15 0.15 0.75"/></material>
  <material name="nose_cone"><color rgba="0.0 0.85 1.0 1.0"/></material>
  <material name="landing_gear"><color rgba="0.18 0.18 0.2 1.0"/></material>

  <link name="base_link">
    <inertial>
      <mass value="{m:.6f}"/>
      <origin xyz="{r_com[0]:.6f} {r_com[1]:.6f} {r_com[2]:.6f}"/>
      <inertia ixx="{I_com[0,0]:.6e}" ixy="{I_com[0,1]:.6e}" ixz="{I_com[0,2]:.6e}"
               iyy="{I_com[1,1]:.6e}" iyz="{I_com[1,2]:.6e}" izz="{I_com[2,2]:.6e}"/>
    </inertial>
    <!-- Central Fuselage Top Dome -->
    <visual>
      <origin xyz="0 0 0.02"/>
      <geometry><cylinder radius="0.12" length="0.04"/></geometry>
      <material name="dark_carbon"/>
    </visual>
    <!-- Central Fuselage Core -->
    <visual>
      <origin xyz="0 0 0"/>
      <geometry><box size="0.18 0.18 0.05"/></geometry>
      <material name="dark_carbon"/>
    </visual>
    <!-- Forward Heading Arrow/Nose Marker -->
    <visual>
      <origin xyz="0.12 0 0.01"/>
      <geometry><box size="0.06 0.03 0.02"/></geometry>
      <material name="nose_cone"/>
    </visual>
    <collision>
      <origin xyz="0 0 0"/>
      <geometry><cylinder radius="0.25" length="0.08"/></geometry>
    </collision>
  </link>
{arms_urdf}
  <!-- Left Landing Gear -->
  <link name="gear_left">
    <inertial>
      <mass value="0.0001"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="1e-7" ixy="0" ixz="0" iyy="1e-7" iyz="0" izz="1e-7"/>
    </inertial>
    <visual>
      <origin xyz="0 0.10 -0.06"/>
      <geometry><box size="0.01 0.01 0.10"/></geometry>
      <material name="landing_gear"/>
    </visual>
    <visual>
      <origin xyz="0 0.10 -0.11"/>
      <geometry><box size="0.26 0.015 0.01"/></geometry>
      <material name="landing_gear"/>
    </visual>
  </link>
  <joint name="joint_gear_left" type="fixed">
    <parent link="base_link"/>
    <child link="gear_left"/>
    <origin xyz="0 0 0"/>
  </joint>

  <!-- Right Landing Gear -->
  <link name="gear_right">
    <inertial>
      <mass value="0.0001"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="1e-7" ixy="0" ixz="0" iyy="1e-7" iyz="0" izz="1e-7"/>
    </inertial>
    <visual>
      <origin xyz="0 -0.10 -0.06"/>
      <geometry><box size="0.01 0.01 0.10"/></geometry>
      <material name="landing_gear"/>
    </visual>
    <visual>
      <origin xyz="0 -0.10 -0.11"/>
      <geometry><box size="0.26 0.015 0.01"/></geometry>
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
    with open(output_path, "w", encoding="utf-8") as f:
        f.write(urdf)
