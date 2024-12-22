# m2m2_lidar

This package contains the `m2m2_lidar` node, which is responsible for interfacing with the slamtec M2M2 LIDAR sensor in the ROS workspace.

## Overview

The `m2m2_lidar` node reads data from the LIDAR sensor and publishes it to a ROS topic for other nodes to consume.

## Dependencies

nlohmann for json
openssl for decoding byte64

## Build & Installation

It should build as part of the environment biuld, threre is no need to build and install separately.

## Usage

By default this will be used as part of Perseus bringup but if you need to use this node on its own:

1. Source your ROS workspace:

From the ros_ws folder:

```
source install/setup.bash
```

(or adjust to setup.zsh if using zsh)

2. Run the `m2m2_lidar` node:

```
ros2 run sensors m2m2_lidar
```

## Configuration

This is under construction and parameter configuration has not been implemenmted yet but it will be (soon).

At present the node requires that the lidar is available at 192.168.1.243:1445

## Topics

The `m2m2_lidar` node publishes the following topics:

- `/lidar_scan` (sensor_msgs/LaserScan): Contains the LIDAR scan data.

## Contributing

Contributions are welcome! If part of the ROAR team, please talk to Nigel. If you are non part of the ROAR team please open an issue or submit a pull request for any improvements or bug fixes.
