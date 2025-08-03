# Running the Gazebo Simulation

This tutorial provides the basic commands needed to run the Perseus rover in Gazebo simulation.

## Prerequisites

- Nix package manager installed
- Perseus v2 repository cloned locally
- Basic familiarity with command line interface

## Steps

### 1. Enter the Simulation Environment

```bash
nix develop .#simulation
```

This command enters the specialised Nix development shell configured for simulation with all necessary dependencies.

### 2. Navigate to the ROS Workspace

```bash
cd software/ros_ws
```

### 3. Build the Workspace

```bash
colcon build
```

This builds all ROS2 packages in the workspace, including the simulation components.

### 4. Source the Workspace

```bash
source install/setup.bash
```

This sets up the ROS2 environment variables to use the built packages.

### 5. Launch the Simulation

```bash
ros2 launch perseus_simulation perseus_sim.launch.py
```

This command launches the Gazebo simulation with the Perseus rover model.

## Controlling the Rover

Once the simulation is running, you can control the rover using keyboard input. In a **separate terminal**, run:

```bash
nix develop .#simulation
cd software/ros_ws
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

:::

- The first time you run the simulation, Gazebo may download the lunar landscape model, which can take up to 20 minutes
- Ensure you have an internet connection for the initial setup
- The simulation requires significant system resources and may perform better with hardware acceleration

:::
