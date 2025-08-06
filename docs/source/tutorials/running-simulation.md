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

### 6. Activate the Drive Controller

After the simulation has fully started, you need to activate the drive controller. In a **separate terminal**, run:

```bash
nix develop .#simulation
cd software/ros_ws
source install/setup.bash
ros2 control set_controller_state diff_drive_base_controller active
```

This step is required to enable the rover's wheel movement in simulation.

## Controlling the Rover

Once the simulation is running and the controller is active, you can control the rover using keyboard input. In a **separate terminal**, run:

```bash
nix develop .#simulation
cd software/ros_ws
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_out
```

## Troubleshooting

### Rover Not Moving in Teleop

If the rover doesn't respond to keyboard commands:

1. **Check if the controller is active**:

   ```bash
   ros2 control list_controllers
   ```

   You should see `diff_drive_base_controller` as `active`. If it shows `inactive`, run:

   ```bash
   ros2 control set_controller_state diff_drive_base_controller active
   ```

2. **Verify topic connections**:

   ```bash
   ros2 topic info /cmd_vel_out
   ```

   This should show both publishers and subscribers.

3. **Test topic communication**:
   ```bash
   ros2 topic echo /cmd_vel_out
   ```
   You should see messages when pressing keys in the teleop terminal.

:::

- The first time you run the simulation, Gazebo may download the lunar landscape model, which can take up to 20 minutes
- Ensure you have an internet connection for the initial setup
- The simulation requires significant system resources and may perform better with hardware acceleration
- If the rover doesn't move, make sure the drive controller is activated (step 6)

:::
