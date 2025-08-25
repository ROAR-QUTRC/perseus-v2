# Autonomy and Mapping Operations Manual

## 2026 Team Members

Nigel

Lachlan Ikeguchi

## Map Generation Using SLAM

### Prerequisites

Before proceeding with mapping operations, ensure:

1. Perseus robot system is operational
2. SSH access to Perseus is established and functional
3. Your development laptop:
   - Has the Perseus software stack installed
   - Is connected to the same network as Perseus
4. A single M2M2 LiDAR unit is connected to Perseus

### M2M2 LiDAR Configuration

#### IP Address Configuration

1. Connect the M2M2 LiDAR to:
   - Ethernet port
   - 5V power supply
2. Determine the LiDAR's IP address using either:
   - Network scan utility
   - UniFi console interface

Note: This documentation uses 192.168.1.137 as an example IP address. Replace this with your actual LiDAR IP address.

### Perseus System Configuration

Execute the following commands on the Perseus system:

```console
cd perseus-v2
nix run .#ros2 -- run perseus_sensors m2m2_lidar --ros-args -p sensor_ip:=192.168.1.137 -p sensor_port:=1446
```

Technical Note: The M2M2 LiDAR utilises port 1446 by default. This port can be reconfigured through the M2M2's web administration interface if required.

#### Verification

Verify LiDAR operation by:

1. Monitoring terminal output for expected messages
2. Confirming scan topic presence:
   ```console
   nix run .#ros2 -- topic list
   ```

### Development Laptop Configuration

Execute these commands in a new terminal session on your development laptop:

```console
cd perseus-v2
nix run .#ros2 -- launch autonomy mapping_using_slam_toolbox.launch.py
```

This sequence launches RViz2, providing visualisation of Perseus and the developing map.

### Technical Notes

- Map updates occur only after Perseus has executed sufficient movement or rotation to trigger an update
- Update trigger parameters are configurable in `config/slam_toolbox_params.yaml`
- The system utilises ROS2's SLAM Toolbox for mapping functionality

## Mapping & Autonomous Task - Australian Rover Challenge 2025

- **Goal:** Autonomous exploration and mapping, navigation to specific landmarks by Perseus.

#### Points break-down

| Activity                                  | Points                  |
| ----------------------------------------- | ----------------------- |
| - Leave the Start Area Autonomously       | 5 points                |
| - For each placard imaged and relayed     | 6 points per placard    |
| - Location within 300mm of true position  | 5 points per cube       |
| - Location within 600mm of true position  | 2 points per cube       |
| - Autonomous phase bonus                  | Double the above points |
| - Design and justification for navigation | Up to 5 points          |
| - Mapping system design                   | Up to 5 points          |
| - Details and visualisation of the map    | Up to 15 points         |
| **Total Possible Points**                 | 100 points              |

### Autonomous Phase

- **Start Condition:** Rover must autonomously exit the start area for points.
- **Navigation:**
  - Task: Navigate to five placards using a pre-provided schematic.
  - **Points:** 6 points per placard imaged and relayed to judges.
- **Rules:**
  - No manual control once rover begins moving.
  - Interventions move to non-autonomous phase.

### Non-Autonomous Phase

- Teams can take manual control anytime, for further exploration or troubleshooting which ends the ability to gather points in the autonomous phase.

### Exploratory Mapping

- **Objective:** Locate four 100x100x100mm cubes (red, green, blue, white).
- **Points:**
  - 5 points for each cube located within 300mm accuracy.
  - 2 points if within 600mm.
  - Double points if reported during autonomous phase.

#### Data and Mapping Restrictions

- All mapping data must be gathered during the task; no prior arena knowledge allowed.

#### Presentation

- **Autonomous Navigation Design:**
  - Discuss the autonomous system's design, advantages, and limitations.
- **Mapping System Design:**
  - Explain mapping navigation methods.
  - Justify autonomy level, map format, and feature choices.
- **Map Visualisation:**
  - Present arena map, judged for coverage, completeness, resolution, and accuracy.
- **Points:** Up to 25 points for the quality of the presentation.

### Scoring and Penalties

- Points for navigation success, cube location accuracy, and presentation.
- Penalties for autonomous phase collisions or exiting arena requiring E-STOP activation.

This task emphasises autonomous operation, navigation, and mapping, with a focus on practical application of robotics in space exploration scenarios.
