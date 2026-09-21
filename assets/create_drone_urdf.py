"""Test and prototype improved drone visualization."""

import numpy as np
import pybullet as p
import pybullet_data


def create_hexacopter_urdf(output_path: str):
    """Generate a clean, high-detail multicopter URDF for PyBullet."""
    urdf = """<?xml version="1.0"?>
<robot name="hexacopter">
  <material name="dark_carbon"><color rgba="0.12 0.12 0.14 1.0"/></material>
  <material name="arm_metal"><color rgba="0.25 0.25 0.28 1.0"/></material>
  <material name="motor_silver"><color rgba="0.75 0.75 0.78 1.0"/></material>
  <material name="prop_front"><color rgba="0.1 0.8 0.2 0.7"/></material>
  <material name="prop_rear"><color rgba="0.85 0.2 0.1 0.7"/></material>
  <material name="nose_cone"><color rgba="0.0 0.8 1.0 1.0"/></material>
  <material name="landing_gear"><color rgba="0.18 0.18 0.2 1.0"/></material>

  <!-- Base Link / Central Hub -->
  <link name="base_link">
    <inertial>
      <mass value="3.646"/>
      <origin xyz="0 0 -0.00229"/>
      <inertia ixx="0.04092" ixy="5.656e-5" ixz="1.313e-5" iyy="0.04017" iyz="-6.494e-5" izz="0.06921"/>
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
      <geometry><cylinder radius="0.15" length="0.06"/></geometry>
    </collision>
  </link>

  <!-- Macro for 6 arms, motors, and rotor blades -->
"""
    # 6 arms at 60 degree intervals: 0, 60, 120, 180, 240, 300
    arm_angles = [0.0, 60.0, 120.0, 180.0, 240.0, 300.0]
    arm_length = 0.28
    for i, deg in enumerate(arm_angles, start=1):
        rad = np.radians(deg)
        cos_a = np.cos(rad)
        sin_a = np.sin(rad)
        # Arm midpoint
        arm_x = (arm_length / 2.0) * cos_a
        arm_y = (arm_length / 2.0) * sin_a
        # Motor position
        m_x = arm_length * cos_a
        m_y = arm_length * sin_a
        prop_mat = "prop_front" if (deg <= 60 or deg >= 300) else "prop_rear"

        urdf += f"""
  <!-- Arm {i} -->
  <link name="arm_{i}">
    <inertial>
      <mass value="0.001"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
    </inertial>
    <visual>
      <origin xyz="{arm_x:.5f} {arm_y:.5f} 0.0" rpy="0 1.5707 {rad:.5f}"/>
      <geometry><cylinder radius="0.012" length="{arm_length:.4f}"/></geometry>
      <material name="arm_metal"/>
    </visual>
    <!-- Motor {i} -->
    <visual>
      <origin xyz="{m_x:.5f} {m_y:.5f} 0.025"/>
      <geometry><cylinder radius="0.022" length="0.035"/></geometry>
      <material name="motor_silver"/>
    </visual>
    <!-- Propeller Disc {i} -->
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

    # Add landing gear skids
    urdf += """
  <!-- Left Landing Gear -->
  <link name="gear_left">
    <inertial>
      <mass value="0.001"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
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
    <collision>
      <origin xyz="0 0.10 -0.11"/>
      <geometry><box size="0.26 0.015 0.01"/></geometry>
    </collision>
  </link>
  <joint name="joint_gear_left" type="fixed">
    <parent link="base_link"/>
    <child link="gear_left"/>
    <origin xyz="0 0 0"/>
  </joint>

  <!-- Right Landing Gear -->
  <link name="gear_right">
    <inertial>
      <mass value="0.001"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
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
    <collision>
      <origin xyz="0 -0.10 -0.11"/>
      <geometry><box size="0.26 0.015 0.01"/></geometry>
    </collision>
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


if __name__ == "__main__":
    create_hexacopter_urdf("assets/hexacopter.urdf")
    p.connect(p.DIRECT)
    uav = p.loadURDF("assets/hexacopter.urdf")
    print("Hexacopter URDF created and loaded successfully into PyBullet! Joints:", p.getNumJoints(uav))
    p.disconnect()
