# Launch File Reference

## perseus.launch.py (Base System)

**Package:** `perseus`

**Starts:**

- Robot State Publisher
- DiffDriveController
- JointStateBroadcaster
- Twist Mux
- Rosbridge WebSocket server

**Required Inputs:** None (initialises hardware)

**Outputs:**

| Topic/TF             | Description                    |
| -------------------- | ------------------------------ |
| `/odom`              | Wheel odometry                 |
| `/joint_states`      | Joint positions and velocities |
| `odom` → `base_link` | Odometry TF (if EKF disabled)  |

**Arguments:**

| Argument            | Default                               | Options           |
| ------------------- | ------------------------------------- | ----------------- |
| `use_sim_time`      | `False`                               | `True`, `False`   |
| `use_mock_hardware` | `False`                               | `True`, `False`   |
| `hardware_plugin`   | `perseus_hardware/VescSystemHardware` | See below         |
| `can_bus`           | `can0`                                | Any CAN interface |
| `payload`           | `""`                                  | `bucket`, `""`    |

**Hardware Plugin Options:**

- `perseus_hardware/VescSystemHardware` — Real VESC motors
- `perseus_hardware/McbSystemHardware` — MCB controller
- `gz_ros2_control/GazeboSimSystem` — Gazebo simulation
- `mock_components/GenericSystem` — Testing

**Verification:**

```console
ros2 topic echo /odom --once
ros2 topic echo /joint_states --once
ros2 run tf2_tools view_frames
```

---

## mapping_using_slam_toolbox.launch.py

**Package:** `autonomy`

**Starts:**

- SLAM Toolbox (sync mode)
- RViz2 with mapping configuration

**Required Inputs:**

| Input                | Source                     |
| -------------------- | -------------------------- |
| `/scan`              | LiDAR driver               |
| `odom` → `base_link` | EKF or DiffDriveController |

**Outputs:**

| Output         | Description        |
| -------------- | ------------------ |
| `/map`         | Occupancy grid     |
| `map` → `odom` | SLAM correction TF |

**Verification:**

```console
ros2 topic echo /map --once
ros2 run tf2_ros tf2_echo map odom
```

---

## perseus_nav_bringup.launch.py

**Package:** `autonomy`

**Starts:** Full Nav2 stack

**Lifecycle Nodes:**

- `controller_server`
- `smoother_server`
- `planner_server`
- `behavior_server`
- `velocity_smoother`
- `collision_monitor`
- `bt_navigator`
- `waypoint_follower`
- `docking_server`

**Required Inputs:**

| Input                  | Source              |
| ---------------------- | ------------------- |
| `/scan`                | LiDAR driver        |
| `/map`                 | Map server or SLAM  |
| `/odometry/filtered`   | EKF                 |
| All required TF frames | RSP, EKF, AMCL/SLAM |

**Outputs:**

| Output              | Description                  |
| ------------------- | ---------------------------- |
| `/cmd_vel_nav`      | Navigation velocity commands |
| `/plan`             | Global path                  |
| `/local_costmap/*`  | Local obstacle map           |
| `/global_costmap/*` | Global planning map          |

**Arguments:**

| Argument          | Default                   | Description                   |
| ----------------- | ------------------------- | ----------------------------- |
| `use_sim_time`    | `false`                   | Use simulation clock          |
| `autostart`       | `true`                    | Auto-activate lifecycle nodes |
| `params_file`     | `perseus_nav_params.yaml` | Navigation parameters         |
| `use_composition` | `False`                   | Use composable nodes          |

**Verification:**

```console
ros2 lifecycle list /controller_server
ros2 lifecycle list /bt_navigator
ros2 service call /controller_server/get_state lifecycle_msgs/srv/GetState
```

---

## robot_state_publisher.launch.py

**Package:** `perseus`

**Starts:** Robot State Publisher only

**Outputs:**

- `/robot_description` — URDF string
- `/tf_static` — Static transforms from URDF
- `/tf` — Joint state transforms

**Arguments:**

| Argument          | Default  | Description               |
| ----------------- | -------- | ------------------------- |
| `use_sim_time`    | `False`  | Use simulation clock      |
| `hardware_plugin` | Required | Hardware interface plugin |
| `can_bus`         | `can0`   | CAN bus interface         |

---

## controllers.launch.py

**Package:** `perseus`

**Starts:**

- Controller Manager
- DiffDriveController
- JointStateBroadcaster

**Outputs:**

- `/odom` — Wheel odometry
- `/joint_states` — Joint feedback

---

## handheld_mapping.launch.py

**Package:** `autonomy`

**Starts:** SLAM Toolbox configured for handheld operation

**Use Case:** Mapping without the rover platform (e.g., LiDAR on a pole)

---

## cmd_vel_mux.launch.py

**Package:** `autonomy`

**Starts:** Velocity command multiplexer

**Function:** Arbitrates between `/cmd_vel_nav` and other velocity sources based on priority and timeout.
