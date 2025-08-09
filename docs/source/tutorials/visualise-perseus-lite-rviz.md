# Visualise Perseus-Lite in RViz

This tutorial shows how to visualise the Perseus-Lite robot model in RViz2, allowing you to inspect the TF tree, joint orientations, and robot structure.

## Prerequisites

- Nix development environment set up
- ROS 2 workspace built
- Perseus-Lite packages installed

## Steps

### 1. Enter the Development Environment

```bash
nix develop
```

### 2. Build the Workspace

Navigate to the ROS workspace and build the Perseus-Lite description package:

```bash
cd software/ros_ws
colcon build --packages-select perseus_lite_description
```

### 3. Source the Workspace

```bash
source install/setup.bash
```

### 4. Launch the Visualisation

Run the launch file with the nixgl wrapper for OpenGL support:

```bash
ros2 launch perseus_lite_description view_robot_nixgl.launch.py
```

## What You'll See

The launch file starts three components:

1. **Robot State Publisher** - Publishes TF transforms based on the URDF model
2. **Joint State Publisher GUI** - Provides sliders to manually control joint positions
3. **RViz2** - Displays the robot model and TF tree

In RViz2, you'll see:

- The Perseus-Lite robot model rendered with meshes
- TF frames shown as axes (0.5m scale) with labels
- A grid for spatial reference
- Base axes indicator at the robot's centre

## Interacting with the Model

Use the Joint State Publisher GUI window to:

- Move individual joints using the sliders
- Verify joint rotation directions
- Check joint limits and ranges
- Test the rocker arm suspension movement

This visualisation is particularly useful for:

- Debugging URDF joint definitions
- Verifying coordinate frame orientations
- Understanding the robot's kinematic structure
- Testing joint configurations before hardware deployment
