# Quick Start

## Prerequisites

- ROS2 Jazzy environment via Nix
- Perseus workspace cloned and built
- Network connectivity between development laptop and Perseus

## Bring-Up Order

The autonomy stack must be started in the following order:

```text
1. Base Hardware / Drivers (perseus.launch.py)
   └── Robot State Publisher, Controllers, Twist Mux

2. Sensors
   └── LiDAR (m2m2_lidar node)
   └── IMU (i2c_imu driver - if not integrated)

3. Localisation
   └── EKF (robot_localization)
   └── AMCL (when using pre-built map)

4. SLAM (mapping_using_slam_toolbox.launch.py)
   └── OR load existing map via map_server

5. Navigation (perseus_nav_bringup.launch.py)
   └── Nav2 stack with behaviour trees
```

## Commands

### Real Robot — Full Autonomy Stack

#### 1. Perseus (big-brain) SSH:

**Terminal 1.1 — Perseus (SSH)**

```console
cd perseus-v2
nix run
```

**Terminal 1.2 — Perseus (SSH)**

```console
cd perseus-v2
nix shell
nix run .#ros2 -- launch autonomy autonomy.launch.py
```

**Terminal 1.3 — Perseus (SSH)**

```console
cd perseus-v2/software/web-ui
nix shell
yarn # This installs dependencies
yarn build # Build the server
yarn start # Runs the built server on port 3000
```

**Terminal 1.4 — Perseus (SSH)**

```console
cd perseus-v2/software/ros_ws
nix shell
source install/setup.sh
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

**Terminal 1.5 — Perseus (SSH)**

```console
cd perseus-v2/software/ros_ws
nix shell
source install/setup.sh
ros2 launch perseus_autonomy_bridge nav2_waypoints_bridge.launch.py
```

#### 2. Perseus (medium-brain) SSH:

**Terminal 2.1 — Development Laptop**

```console
cd perseus-v2
nix run .#ros2 -- launch autonomy mapping_using_slam_toolbox.launch.py
```

**Terminal 2.2 — Development Laptop (Navigation)**

```console
cd perseus-v2
nix run .#ros2 -- launch autonomy perseus_nav_bringup.launch.py
```

### Simulation

```console
cd perseus-v2
nix run .#ros2 -- launch perseus perseus.launch.py use_sim_time:=True hardware_plugin:=gz_ros2_control/GazeboSimSystem
```

### Recording (rosbag)

```console
nix run .#ros2 -- bag record /scan /odom /imu/data /tf /tf_static /odometry/filtered
```

### Debug Minimal Stack (TF and Odometry Only)

```console
nix run .#ros2 -- launch perseus robot_state_publisher.launch.py
nix run .#ros2 -- launch perseus controllers.launch.py
```

## M2M2 LiDAR Configuration

### IP Address Configuration

1. Connect the M2M2 LiDAR to:
   - Ethernet port
   - 5V power supply
2. Determine the LiDAR's IP address using either:
   - Network scan utility
   - UniFi console interface

:::{note}
This documentation uses `192.168.1.137` as an example IP address. Replace with your actual LiDAR IP.
:::

### Verification

Verify LiDAR operation by:

1. Monitoring terminal output for expected messages
2. Confirming scan topic presence:
   ```console
   nix run .#ros2 -- topic list | grep scan
   nix run .#ros2 -- topic hz /scan
   ```

## Technical Notes

- Map updates occur only after Perseus has executed sufficient movement or rotation
- Update trigger parameters are configurable in `config/slam_toolbox_params.yaml`
- The system uses ROS2 SLAM Toolbox for mapping functionality
