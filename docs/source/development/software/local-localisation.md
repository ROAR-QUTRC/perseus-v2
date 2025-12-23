# Local Localisation

## Introduction

Local robot localisation refers to estimating a robot’s position and orientation **relative to its initial reference frame**, rather than to a global map. In our system, the frame **`/odom`** acts as the origin of the robot’s local coordinate system and is initialised when the ROS2 controller starts. All subsequent motion updates are expressed relative to this frame.

A variety of sensing modalities can be used to estimate odometry, including **wheel encoders**, **IMUs**, **cameras**, **GPS**, and **indoor motion-capture systems** (e.g., OptiTrack). Each of these sensors introduces uncertainties—such as drift, noise, or environmental disturbances—making _perfect_ localisation impossible. Instead, an appropriate odometry method is selected based on the available hardware, mission constraints, and operating environment to achieve a localisation estimate that is sufficiently accurate, stable, and computationally feasible for navigation tasks.

## Overview

For a Lunar Rover operating on the Moon, several localisation techniques commonly used on Earth are **not feasible**. For example:

- **GPS cannot be used**, as there is no satellite navigation infrastructure on the Moon.
- **OptiTrack or other external motion-capture systems are unsuitable** due to the absence of controlled indoor environments.
- **Magnetometers perform poorly**, as the Moon lacks a global magnetic field and therefore cannot provide reliable heading information.

Given these constraints, the most practical and cost-effective odometry sources for lunar surface operations are:

- **Wheel encoders**, which provide incremental motion estimation through wheel rotation.
- **Inertial Measurement Units (IMUs)**, which estimate orientation and short-term motion but suffer from drift over time.
- **Onboard cameras**, which enable **visual odometry** or **visual-inertial odometry** by tracking visual features across frames to estimate movement.

By combining these complementary sensors—typically through an **Extended Kalman Filter (EKF)** or a similar fusion framework—the rover can maintain a robust, drift-bounded estimate of its pose within the local odometry frame. This hybrid approach provides a balance between accuracy, computational cost, and environmental suitability, making it ideal for lunar surface exploration where external localisation references are unavailable.

Due to the complexity and computational demands of full visual SLAM (VSLAM) pipelines, our current objective is to focus on achieving reliable **EKF-based fusion of IMU and wheel encoder data**. Since no single sensor can provide perfectly accurate odometry—especially in a feature-sparse and unstructured lunar environment—it becomes essential to understand, configure, and fine-tune the EKF parameters. A well-tuned EKF is critical for reducing drift, managing sensor noise, and ensuring that the rover maintains a stable and consistent local pose estimate throughout its mission.

:::{note}

### **Why not just use a single encoder or IMU for odometry estimation?**

Relying on **only one sensor** for odometry—whether it be wheel encoders or an IMU—is insufficient for reliable localisation, especially in a challenging environment like the lunar surface:

**1. Wheel Encoders Alone Are Not Reliable**

- **Slip and wheel sinkage** on loose regolith cause the measured wheel rotation to differ from actual motion.
- **No correction mechanism** exists—errors accumulate without any way to reset or bound them.
- Encoder-only odometry drifts significantly over time, especially during long traverses or rough terrain.

**2. IMU Alone Cannot Maintain Accurate Pose**

- IMUs measure acceleration and angular velocity, which must be **integrated** to estimate position.
- This integration process causes **rapid drift**, as even tiny biases or noise accumulate exponentially.
- Low-cost or space-rated IMUs without external reference can become unusably inaccurate within seconds to minutes.

**3. Fundamental Limitation: No Single Sensor Provides Complete, Drift-Bounded Odometry**
Each sensor has **complementary strengths and weaknesses**:

- Encoders provide **short-term accurate displacement**, but fail when slip occurs.
- IMUs provide **short-term accurate rotation and dynamics**, but drift rapidly in position.

Without sensor fusion, the rover would either:

- **Quickly lose localisation**, or
- **Misinterpret its motion**, leading to incorrect navigation, poor path tracking, or hazard avoidance failures.

**4. Fusion is Required to Achieve Stable Localisation**
By combining IMU and encoder data through an EKF, the system:

- Compensates for the drift of the IMU
- Corrects encoder errors during slip
- Produces a **consistent and bounded** pose estimate
- Supports higher-level navigation algorithms (Nav2, mapping, hazard avoidance, etc.)
  the rest of your documentation.
  :::

## Tuning Parameters

To learn more about configuration options and recommended practices, refer to the official **[`robot_localization` documentation](https://docs.nav2.org/setup_guides/odom/setup_robot_localization.html)**.

A typical configuration for the `ekf_filter_node` is shown below. This setup fuses wheel encoder odometry (from the differential drive controller) and IMU measurements, each with appropriate covariance values, to produce a stable and drift-bounded estimate of the rover’s local odometry.

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    sensor_timeout: 0.1
    two_d_mode: false
    publish_acceleration: false
    publish_tf: true

    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    # Wheel encoder (odometry) input
    odom0: /odom
    odom0_config:
      [
        true,
        true,
        false,
        false,
        false,
        true,
        true,
        true,
        false,
        false,
        false,
        true,
        false,
        false,
        false,
      ]
    odom0_queue_size: 10
    odom0_nodelay: false
    odom0_differential: false
    odom0_relative: false
    odom0_pose_rejection_threshold: 5.0
    odom0_twist_rejection_threshold: 1.0

    # IMU input
    imu0: /imu/data
    imu0_config:
      [
        false,
        false,
        false,
        true,
        true,
        true,
        false,
        false,
        false,
        true,
        true,
        true,
        true,
        true,
        true,
      ]
    imu0_queue_size: 10
    imu0_nodelay: false
    imu0_differential: false
    imu0_relative: true
    imu0_pose_rejection_threshold: 0.8
    imu0_twist_rejection_threshold: 0.8
    imu0_linear_acceleration_rejection_threshold: 0.8
    imu0_remove_gravitational_acceleration: true

    # Process noise model for the EKF
    process_noise_covariance: [... trimmed for readability ...]

    # Initial uncertainty in the state estimate
    initial_estimate_covariance: [1e-9, 0, 0, ...]
```

This configuration ensures that the EKF receives encoder-derived linear velocity and yaw information, along with IMU-derived angular velocity and orientation, weighted appropriately by their covariance values. Together, these inputs produce a fused odometry estimate that is more reliable than either sensor alone.

---

### State Variables

The `*_config` arrays specify which elements of the EKF state vector each sensor contributes to. For example:

```yaml
odom0_config:
  [
    true,
    true,
    false,
    false,
    false,
    true,
    true,
    true,
    false,
    false,
    false,
    true,
    false,
    false,
    false,
  ]
imu0_config:
  [
    false,
    false,
    false,
    true,
    true,
    true,
    false,
    false,
    false,
    true,
    true,
    true,
    true,
    true,
    true,
  ]
```

Each Boolean corresponds to a state variable in the filter’s 15-element state vector.

Nice, this is shaping up really well. Here’s a **compact state-space section** you can drop under **“State Variables”** (and/or after the tuning part).

#### **15 Element State Space Matrices:**

The EKF in `robot_localization` uses a 15-dimensional state vector:
$ {
\mathbf{x} =
\begin{bmatrix}
x & y & z & \phi & \theta & \psi & v_x & v_y & v_z & \omega_x & \omega_y & \omega_z & a_x & a_y & a_z
\end{bmatrix}^\top
} $
where:

| Index | Symbol                           | Meaning                          |
| ----- | -------------------------------- | -------------------------------- |
| 0–2   | $(x, y, z)$                      | Position in `odom` frame         |
| 3–5   | $(\phi, \theta, \psi)$           | Roll, pitch, yaw                 |
| 6–8   | $(v_x, v_y, v_z)$                | Linear velocities in `odom`      |
| 9–11  | $(\omega_x, \omega_y, \omega_z)$ | Angular velocities (body / odom) |
| 12–14 | $(a_x, a_y, a_z)$                | Linear accelerations             |

In our case, we let the **IMU provide orientation, angular velocity, and linear acceleration**, while the **wheel encoders provide linear velocity and yaw-rate information**. By selectively enabling these elements in the `*_config` arrays, each sensor contributes only the components it can measure reliably. The EKF then fuses these complementary measurements into a single, drift-bounded state estimate:

- **Encoders** stabilise the rover’s short-term linear motion and yaw.
- **IMU** stabilises orientation, angular motion, and short-term accelerations.
- **Process and measurement covariances** determine how heavily the EKF trusts each source.

This selective fusion ensures that each state variable in the 15-element vector is updated by the best available sensor, while the EKF model propagates the full state over time. The result is a smooth, consistent, and reliable odometry estimate suitable for local navigation, control, and downstream modules such as Nav2, mapping, or hazard detection.

#### **Tuning `process_noise_covariance` and `initial_estimate_covariance`**

In `robot_localization`, these two matrices control **how the EKF behaves over time**:

- `process_noise_covariance` → how much the filter believes the state can change _between_ measurements.
- `initial_estimate_covariance` → how uncertain the filter is about the **starting pose and state**.

```yaml
# Process noise model for the EKF
process_noise_covariance: [...]

# Initial uncertainty in the state estimate
initial_estimate_covariance: [1e-9, 0, 0, ...]
```

**`process_noise_covariance` (Q)**
Each diagonal element corresponds to one state variable in the 15-element vector ($[x, y, z, \phi, \theta, \psi, v_x, \dots, a_z]$).

- Larger values → EKF assumes the **model is less accurate**, so it trusts **sensor measurements more** and adapts faster.
- Smaller values → EKF assumes the **model is reliable**, so it smooths aggressively and reacts more slowly.

Typical practice:

- Use **higher Q** for velocities and accelerations (they change quickly, are hard to model).
- Use **lower Q** for orientation and position in simulation, where the motion model is clean.
- If `/odometry/filtered` looks **sluggish** → slightly _increase_ the relevant Q terms.
- If it looks **noisy or jittery** → slightly _decrease_ the relevant Q terms.

**`initial_estimate_covariance` (P₀)**
This matrix defines the EKF’s **initial confidence** in each state variable.

- Very small values (e.g. `1e-9`) mean the filter assumes the starting state is **almost exact** (typical for simulation, where `odom` and `base_link` are known).
- Larger values tell the filter it is **uncertain** about the initial pose, so early measurements can shift the estimate more aggressively.

For a Lunar Rover simulation, it is reasonable to:

- Keep **very low covariance** on the initial pose if the robot always spawns at a known location.
- Increase initial covariance only if you deliberately introduce uncertainty (e.g., randomised spawn poses or noisy initial alignment).

In practice, you tune these two matrices together:
`process_noise_covariance` shapes how the state evolves over time, while `initial_estimate_covariance` controls how quickly the EKF “locks in” to the correct pose at startup.

:::{note}

#### **Practical Tuning Workflow Summary**

- Start with **moderate process noise (Q)** for velocities and accelerations.
- Tune **IMU covariances first** to stabilise orientation and angular velocity.
- Tune **encoder covariances next** to achieve smooth linear velocity and yaw estimates.
- Adjust based on behaviour:

  - **Jittering / jumping** → increase measurement covariance.
  - **Lagging / slow response** → decrease measurement covariance.
  - **Oscillation / divergence** → increase process noise.

- Validate tuning by driving the rover and monitoring:

  - `/odometry/filtered`
  - raw `/odom`
  - raw `/imu`
    :::
