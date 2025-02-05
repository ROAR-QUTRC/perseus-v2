# Autonomy & Mapping

## Using mapping

This guide assumes that you have followed the guide to Perseus Teleoperation and are able to teleoperate Perseus.

This guide assumes that:

- Perseus is on and working
- You have ssh'd into Perseus and are able to execute commands on Perseus
- You have a laptop with the Perseus software stack running on the same network as Perseus.
- only one m2m2 lidar is attached.

To create a map whilst moving Perseus through the environment you will need to have the m2m2 lidar attached, publishing the laser scan points and launch mapping which will start the slam toolbox and open rviz2 to visualise the map.

## Determine the IP address of the m2m2 lidar

With the m2m2 lidar plugged into ethernet, and provided 5V use a scan or the Unifi console to determine the IP address of the m2m2 lidar. For the purpose of this guide it is assumed that the IP address is 192.168.1.137 but use the actual address found.

## Commands to run on Perseus

```console
cd /perseus-v2/software/ros_ws
colcon build
source install/setup.bash
ros2 run perseus_sensors m2m2_lidar --ros-args -p sensor_ip:=192.168.1.137 -p sensor_port:=1446
```

:::{note}
Port 1446 is the default port that the m2m2 uses, this can be changed via the web admin functionality of the m2m2 lidar.
:::

You can check the m2m2 lidar is operating as expected by monitoring the terminal messages as well as viewing the available topics via:

```console
ros2 topic list
```

Look for the scan topic.

## Commands to run on Laptop

Run the following in a new terminal on the Laptop:

```console
cd /perseus-v2/software/ros_ws
colcon build
source install/setup.bash
ros2 launch autonomy mapping_using_slam_toolbox.launch.py
```

Amongst other things, this will launch rviz2 showing Perseus and the map being created.

Note that the map will only update once Perseus has moved or turned sufficiently far to trigger and update. Parameters including these trigger values are found in the file `config/slam_toolbox_params.yaml`.

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
