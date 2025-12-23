# Robot Description

:::{note}
URDF (Unified Robot Description Format) is an XML-based file format, central to the Robot Operating System (ROS) and other robotics tools, used to describe a robot's physical structure, including its rigid bodies (links) and how they connect and move (joints), defining its kinematics, visuals, and physical properties for simulation and control. It acts as a digital blueprint, detailing each part's geometry, mass, and relationship, making robots understandable and usable in software like Gazebo and RViz [^1].
:::

## Overview

The Perseus Rover URDF (Unified Robot Description Format) is rebuilt to provide a complete, modular, and simulation-ready description of the rover’s physical structure, joints, sensors, and coordinate frames. It is designed to support RViz visualisation, Gazebo Sim simulation, and Nav2-based navigation, following **[REP-103](https://www.ros.org/reps/rep-0103.html)[^2]** conventions and ROS2 best practices. The URDF is built using Xacro macros to improve maintainability and allow rapid redesign of components such as sensors, drivetrain, and chassis geometry.

## Repository Structure

The [`perseus_description`](https://github.com/ROAR-QUTRC/perseus-v2/tree/main/software/ros_ws/src/perseus_description) package contains the full URDF/Xacro model of the Perseus Rover.
It provides a central source of truth for the robot’s physical structure, and is shared across other packages such as simulation, control, perception, and navigation.

Below is the structure:

```
perseus_description/
 ├── launch/
 │    └── view_perseus.launch.py               # Launch file to visualise the URDF in RViz
 │
 ├── meshes/                                    # Visual/collision 3D models used by URDF
 │    ├── chassis.dae
 │    ├── differential_bar.dae
 │    ├── dozer.dae
 │    ├── flange_bearing.dae
 │    ├── gearbox.dae
 │    ├── rocker_left.dae
 │    ├── rocker_right.dae
 │    └── wheel.dae
 │
 ├── ros2_control/
 │    └── perseus.ros2_control.xacro           # Hardware interface definition for ros2_control
 │
 ├── rviz/
 │    ├── view_odom.rviz                       # RViz configuration for odometry view
 │    └── view_perseus.rviz                    # RViz configuration for robot model inspection
 │
 ├── urdf/                                      # All Xacro modules that compose the robot
 │    ├── camera_depth.urdf.xacro              # Depth camera frame and model
 │    ├── chassis.urdf.xacro                   # Main chassis link definition
 │    ├── flange_bearing.urdf.xacro            # Bearing component used in joints
 │    ├── inertia_macros.xacro                 # Standard inertia & mass macros
 │    ├── motor_wheel.urdf.xacro               # Motor + wheel assembly macros
 │    ├── perseus.materials.xacro              # Material/color definitions
 │    ├── perseus.urdf.xacro                   # Main combined URDF entry point
 │    ├── rocker.urdf.xacro                    # Rocker suspension mechanism
 │    ├── sensors.urdf.xacro                   # Sensor mounts (2D & 3D Lidar)
 │    └── wheel.urdf.xacro                     # Wheel link geometry + joint
 │
 ├── CMakeLists.txt                            # Package build configuration
 └── package.xml                               # ROS2 package manifest
```

## Robot Architecture

The following diagram illustrates the TF architecture of the Perseus Rover when operating with odometry (`/odom`) estimation enabled. This representation shows how major physical components, drivetrain assemblies, and sensors attach to the `base_link`, which serves as the primary reference frame for state estimation, navigation, and perception.

```dot
    odom -> base_link;
    base_link -> base_footprint;
    base_link -> chassis;

    chassis -> camera_link;
    chassis -> flange_bearing;
    chassis -> left_rocker;
    chassis -> right_rocker;
    chassis -> laser_2d_frame;
    chassis -> laser_frame;

    camera_link -> camera_link_optical;

    flange_bearing -> differential_bar;

    left_rocker -> front_left_motor;
    front_left_motor -> front_left_wheel;

    left_rocker -> rear_left_motor;
    rear_left_motor -> rear_left_wheel;

    right_rocker -> front_right_motor;
    front_right_motor -> front_right_wheel;

    right_rocker -> rear_right_motor;
    rear_right_motor -> rear_right_wheel;

```

## Coordinate Frames (REP-103 Compliance)

The Perseus Rover URDF follows **REP-103** conventions [^2] to ensure consistent coordinate alignment across ROS2 tools such as RViz, Nav2, AMCL, EKF, and perception pipelines. These conventions guarantee that all sensors, wheels, and robot links behave predictably in both simulation and real-world deployments.

### **Base Frame (`base_link`)**

The `base_link` frame defines the central reference for the robot’s body:

- **+X → forward**
- **+Y → left**
- **+Z → upward**

This frame serves as the anchor for most transforms and is used by Nav2, AMCL, and state-estimation frameworks (e.g., `robot_localization`) to interpret the robot’s pose and motion.

### **Wheel Joint Frames**

Each wheel joint follows standard differential-drive and rocker-based rover conventions:

- **Rotation axis:** +Y
- **Forward motion:** +X

This ensures proper behaviour when executing velocity commands through `/cmd_vel_out` and when using `ros2_control` interfaces to apply wheel velocities.

### **IMU Frame**

The IMU link is aligned using the standard aircraft convention:

- **+X → forward**
- **+Y → left**
- **+Z → upward**

Aligning the IMU with the base frame (`/chassis`) simplifies sensor fusion, maintains compatibility with IMU drivers, and prevents orientation mismatches in EKF pipelines.

### **Depth Camera Optical Frame**

The optical frame is defined according to REP-103's camera convention:

- **+Z → forward** (optical axis)
- **+X → right**
- **+Y → down**

This is required for correct handling of projection matrices, point cloud orientation, and depth image interpretation in RViz and perception stacks such as vision-based SLAM or obstacle detection modules.

## Links & Joints Description

The Perseus Rover URDF defines a complete kinematic chain composed of multiple rigid links connected through joints that reflect the rover’s physical articulation and drivetrain mechanics. Each link represents a physical component of the robot—such as the chassis, rocker arms, wheels, and sensor housings—while each joint defines how that component moves relative to others. Together, these form the structural foundation for motion simulation, TF (transform) generation, and controller integration.

### **Chassis Link (`chassis`)**

The chassis acts as the central structural body of the rover and serves as the parent link for most major components. It is attached to the `base_link` and provides mounting frames for sensors, drivetrain assemblies, rocker mechanisms, and auxiliary payloads. The chassis mesh accurately reflects the rover’s physical geometry, while its collision model and inertial properties are simplified for stable simulation.

### **Rocker Mechanism (`left_rocker`, `right_rocker`)**

Each rocker arm is defined as an independent link that attaches to the chassis via a revolute joint. These joints allow limited rotational movement, replicating the passive suspension behavior found in physical rocker-bogie systems. The rockers distribute the rover’s weight and allow the wheels to adapt to uneven terrain, improving stability and traction during traversal.

The rocker links also serve as parent frames for the motor-wheel assemblies on each side of the rover.

### **Differential Bar (`differential_bar`)**

The differential bar connects the two rocker arms, allowing their motion to remain mechanically balanced. This link helps ensure that when one rocker is raised (e.g., driving over an obstacle), the opposite rocker compensates, maintaining overall chassis stability. The URDF models this linkage with appropriate constraints and reference frames to emulate real-world behavior without adding unnecessary simulation complexity.

### **Motor–Wheel Assemblies**

Each wheel is mounted through a motor assembly that consists of two primary links:

1. **Motor Housing Link (`*_motor`)**
   This link represents the physical enclosure of the wheel’s drive motor and acts as the mounting base for each wheel. It attaches to a rocker or chassis via a fixed joint.

2. **Wheel Link (`*_wheel`)**
   The wheel link connects to the motor housing via a continuous joint (revolute joint with unrestricted rotation). This joint allows the wheel to spin freely around its axis due to applied velocity or torque commands.
   The wheel joints follow REP-103 conventions, with rotation aligned along the +Y axis and forward motion along +X.

Together, these links and joints form the drivetrain assembly responsible for locomotion.

### **Sensor Links**

Several links in the URDF represent the mounting frames for perception hardware:

- **Depth Camera (`camera_link`, `camera_link_optical`)**
  `camera_link` defines the physical mounting point on the chassis, while `camera_link_optical` follows the REP-103 optical frame convention.
  This ensures correct orientation of projected depth images and point clouds.

- **Laser Scanner Frames (`laser_2d_frame`, `laser_frame`)**
  These links define the exact positioning of LIDAR sensors. Their orientations are aligned so that scans align correctly with the world frame.

- **IMU Link (`imu_link`)**
  The IMU frame is aligned according to aircraft conventions (+X forward, +Y left, +Z up), enabling seamless integration with state estimation frameworks such as `robot_localization`.

Each sensor link is defined with its own inertial and collision properties only where needed, but in many cases sensors are treated as lightweight attachments with simplified inertias.

### **Joint Types and Their Roles**

The URDF uses several joint types, each selected for a specific physical behaviour:

- **Fixed Joints**
  Used where components are rigidly connected, such as sensors mounted to the chassis or motors affixed to rocker arms. These joints introduce no movement.

- **Revolute/Continuous Joints**
  Used for the wheels and suspension elements. Wheel joints are continuous, enabling unrestricted rotation, while rocker joints may be limited or passive based on design.

These joints define not only kinematic relationships but also how transforms propagate through the TF tree, directly influencing how navigation, perception, and simulation interpret the robot’s geometry.

### **Overall Link–Joint Hierarchy**

All links and joints come together to form a coherent kinematic chain:

```
base_link
 └── chassis
      ├── rocker links
      │     └── motor → wheel assemblies
      ├── sensor links
      └── differential bar
```

This structure ensures a physically consistent model where motion commands, sensor alignment, and transform propagation match the rover’s real-world design.

## Mass, Inertia & Collision Geometry

The mass values used in the Perseus URDF are derived from real measurements of the rover’s physical components to ensure that the simulated dynamics accurately reflect the behavior of the actual system. Corresponding inertia tensors were computed based on the component geometries and are included to provide stable and realistic physics interactions.

To maintain high simulation performance and avoid unnecessary computational load, collision geometry has been deliberately simplified. Instead of using detailed mesh models—which typically contain thousands of small triangles and significantly increase physics processing time—the URDF employs basic geometric primitives such as boxes and cylinders for collision representation. These simplified shapes greatly reduce computational overhead while still providing accurate enough contact behaviour for locomotion, obstacle interaction, and navigation tasks.

Inertia is also simplified by using analytical inertia models for boxes and cylinders. These formulas provide stable, physically plausible approximations and prevent numerical instability in simulation. The inertia matrices are computed using the following standard rigid-body equations:

### Box Inertia (dimensions: width = _w_, height = _h_, depth = _d_)

For a solid rectangular box of mass \( m \):

$$
I_{xx} = \frac{1}{12} m (h^2 + d^2)
$$

$$
I_{yy} = \frac{1}{12} m (w^2 + d^2)
$$

$$
I_{zz} = \frac{1}{12} m (w^2 + h^2)
$$

### Cylinder Inertia (radius = _r_, length = _h_)

For a solid cylinder aligned along the Z-axis:

$$
I_{xx} = I_{yy} = \frac{1}{12} m (3r^2 + h^2)
$$

$$
I_{zz} = \frac{1}{2} m r^2
$$

These simplified inertia equations are used throughout the URDF (via `inertia_macros.xacro`) to maintain consistency, reduce computational overhead, and ensure that the rover behaves predictably when simulated in Gazebo Sim or other physics engines.

## References

[^1]: Foxglove Robotics — _What is URDF (Unified Robot Description Format)?_ https://foxglove.dev/robotics/urdf.

[^2]: REP 103 — Standard Units of Measure and Coordinate Conventions. https://www.ros.org/reps/rep-0103.html
