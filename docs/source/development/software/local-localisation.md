# Local Localisation

## Introduction

Local robot localisation refers to estimating a robot’s position and orientation **relative to its initial reference frame**, rather than to a global map. In our system, the frame **`/odom`** acts as the origin of the robot’s local coordinate system and is initialised when the ROS2 controller starts. All subsequent motion updates are expressed relative to this frame.

A variety of sensing modalities can be used to estimate odometry, including **wheel encoders**, **IMUs**, **cameras**, **GPS**, and **indoor motion-capture systems** (e.g., OptiTrack). Each of these sensors introduces uncertainties—such as drift, noise, or environmental disturbances—making *perfect* localisation impossible. Instead, an appropriate odometry method is selected based on the available hardware, mission constraints, and operating environment to achieve a localisation estimate that is sufficiently accurate, stable, and computationally feasible for navigation tasks.

## Overview

For a Lunar Rover operating on the Moon, several localisation techniques commonly used on Earth are **not feasible**. For example:

* **GPS cannot be used**, as there is no satellite navigation infrastructure on the Moon.
* **OptiTrack or other external motion-capture systems are unsuitable** due to the absence of controlled indoor environments.
* **Magnetometers perform poorly**, as the Moon lacks a global magnetic field and therefore cannot provide reliable heading information.

Given these constraints, the most practical and cost-effective odometry sources for lunar surface operations are:

* **Wheel encoders**, which provide incremental motion estimation through wheel rotation.
* **Inertial Measurement Units (IMUs)**, which estimate orientation and short-term motion but suffer from drift over time.
* **Onboard cameras**, which enable **visual odometry** or **visual-inertial odometry** by tracking visual features across frames to estimate movement.

By combining these complementary sensors—typically through an **Extended Kalman Filter (EKF)** or a similar fusion framework—the rover can maintain a robust, drift-bounded estimate of its pose within the local odometry frame. This hybrid approach provides a balance between accuracy, computational cost, and environmental suitability, making it ideal for lunar surface exploration where external localisation references are unavailable.

Due to the complexity and computational demands of full visual SLAM (VSLAM) pipelines, our current objective is to focus on achieving reliable **EKF-based fusion of IMU and wheel encoder data**. Since no single sensor can provide perfectly accurate odometry—especially in a feature-sparse and unstructured lunar environment—it becomes essential to understand, configure, and fine-tune the EKF parameters. A well-tuned EKF is critical for reducing drift, managing sensor noise, and ensuring that the rover maintains a stable and consistent local pose estimate throughout its mission.

:::{note}
### **Why not just use a single encoder or IMU for odometry estimation?**

Relying on **only one sensor** for odometry—whether it be wheel encoders or an IMU—is insufficient for reliable localisation, especially in a challenging environment like the lunar surface:

**1. Wheel Encoders Alone Are Not Reliable**

* **Slip and wheel sinkage** on loose regolith cause the measured wheel rotation to differ from actual motion.
* **No correction mechanism** exists—errors accumulate without any way to reset or bound them.
* Encoder-only odometry drifts significantly over time, especially during long traverses or rough terrain.

**2. IMU Alone Cannot Maintain Accurate Pose**

* IMUs measure acceleration and angular velocity, which must be **integrated** to estimate position.
* This integration process causes **rapid drift**, as even tiny biases or noise accumulate exponentially.
* Low-cost or space-rated IMUs without external reference can become unusably inaccurate within seconds to minutes.

**3. Fundamental Limitation: No Single Sensor Provides Complete, Drift-Bounded Odometry**
Each sensor has **complementary strengths and weaknesses**:

* Encoders provide **short-term accurate displacement**, but fail when slip occurs.
* IMUs provide **short-term accurate rotation and dynamics**, but drift rapidly in position.

Without sensor fusion, the rover would either:

* **Quickly lose localisation**, or
* **Misinterpret its motion**, leading to incorrect navigation, poor path tracking, or hazard avoidance failures.

**4. Fusion is Required to Achieve Stable Localisation**
By combining IMU and encoder data through an EKF, the system:

* Compensates for the drift of the IMU
* Corrects encoder errors during slip
* Produces a **consistent and bounded** pose estimate
* Supports higher-level navigation algorithms (Nav2, mapping, hazard avoidance, etc.)
 the rest of your documentation.
:::