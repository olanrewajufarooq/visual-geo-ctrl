# PyBullet vs MATLAB Robotics System Toolbox: Physical Differences & Equivalence

This document outlines the intentional physical, numerical, and structural differences between the **PyBullet** simulation backend in `adaptive-geo-ctrl-pybullet` and the **MATLAB Robotics System Toolbox** reference in `adaptive-geo-ctrl`.

---

## 1. Multibody Architecture & Floating Base

| Feature | MATLAB Reference (`adaptive-geo-ctrl`) | PyBullet Implementation (`adaptive-geo-ctrl-pybullet`) |
| :--- | :--- | :--- |
| **Model Type** | `rigidBodyTree` with a 6-DoF floating base joint. | Multibody rigid vehicle loaded via dynamically generated URDF with `flags=p.URDF_MERGE_FIXED_LINKS`. |
| **Inertial Mapping** | Direct compound parameter vector $\pi \in \mathbb{R}^{10}$ assigned to the base body. | Exact mass $m$, Center of Gravity offset $r_{\mathrm{com}} = h/m$, and rotated principal moments of inertia derived analytically from $\pi$ and written to URDF. |
| **Link Merging** | Virtual rigid tree traversal. | Fixed visual links (arms, landing gear) merged directly into `base_link` for maximal numerical efficiency without joint friction. |

---

## 2. Physical Wrench & Dynamics Application

- **Body Wrench Convention**: In the paper, control wrench $W = [\tau_b; f_b] \in \mathbb{R}^6$ is defined in the body-fixed link frame attached to the drone's center.
  - In PyBullet, this is applied using:
    ```python
    p.applyExternalForce(uav_id, -1, forceObj=f_b, posObj=[0,0,0], flags=p.LINK_FRAME)
    p.applyExternalTorque(uav_id, -1, torqueObj=tau_b, flags=p.LINK_FRAME)
    ```
- **Damping Compensation**: PyBullet applies small default linear and angular damping ($0.04$) to floating bodies. To ensure exact mathematical equivalence to the paper's unforced Euler-Poincaré equations:
  ```python
  p.changeDynamics(uav_id, -1, linearDamping=0.0, angularDamping=0.0)
  ```

---

## 3. Dynamic Payload Attachment & Detachment

- **MATLAB Reference**: Replaces active mass/inertia parameters in the rigid-body definition at $t = t_{\mathrm{release}}$.
- **PyBullet Implementation**: Models the payload as an independent rigid cuboid body attached to the vehicle via a rigid fixed joint constraint (`p.createConstraint`). At $t = t_{\mathrm{release}}$, `p.removeConstraint` is called.
  - The detached payload falls under gravity, bounces off the ground plane, and interacts with the environment.
  - The vehicle experiences true physical reaction forces during release.

---

## 4. Numerical Integration

- **Timestep**: Fixed at 500 Hz ($\Delta t = 0.002\text{ s}$) matching the MATLAB plant timestep.
- **Multirate Execution**: Controller runs at 50 Hz ($\Delta t = 0.02\text{ s}$, zero-order hold), adaptation runs at 100 Hz ($\Delta t = 0.01\text{ s}$), and physical plant advances at 500 Hz.
- **Integrator**: PyBullet uses a semi-implicit symplectic Euler scheme with an LCP projected Gauss-Seidel constraint solver.

---

## 5. Equivalence Verification

All automated tests in `tests/test_pybullet_plant.py` confirm:
1. Exact hover force balance: commanded vertical wrench $f_z = m g$ keeps vertical acceleration $< 10^{-6}\text{ m/s}^2$.
2. Free fall acceleration under gravity equals $9.81\text{ m/s}^2 \pm 10^{-6}$.
3. Compound $\pi$ calculation between drone and payload matches parallel-axis theorem formulas to within $10^{-12}$.
