# Perseus Sensors - IMU and Topic Nodes

This package provides ROS2 nodes for **IMU bias estimation and correction**, as well as **dynamic topic remapping** for downsampling sensor data.

Included nodes:

1. **BiasEstimator** – estimates IMU gyro bias when the robot is stationary.
2. **BiasRemover** – removes the estimated bias from IMU readings.
3. **TopicRemapper** – dynamically detects a topic type and republishes it at a lower frequency.

---

> ⚠️ **Open-Source Attribution:**  
> The **BiasEstimator** and **BiasRemover** ROS2 nodes in this package are adapted from the
> [ros-perception/imu_pipeline](https://github.com/ros-perception/imu_pipeline/tree/ros2) project.  
> In particular, the bias removal logic is based on the implementation in `imu_processors/src/imu_bias_remover.cpp` from that repository, with extensions for stationary detection, configurable parameters, and composable node support in the Perseus platform.

## 1. BiasEstimator Node

**Node Name:** `imu_bias_estimator`
**Package:** `perseus_sensors`
**Plugin (Composable):** `imu_processors::BiasEstimator`

### Description

Estimates the bias of the IMU angular velocity (`gyro`) when the robot is stationary. Stationary detection can use **odom**, **cmd_vel**, or both.

### Subscribed Topics

| Topic           | Type                                    | Description                                        |
| --------------- | --------------------------------------- | -------------------------------------------------- |
| `imu_in_topic`  | `sensor_msgs/Imu`                       | Raw IMU data                                       |
| `cmd_vel_topic` | `geometry_msgs/Twist` or `TwistStamped` | (Optional) Robot velocity for stationary detection |
| `odom_topic`    | `nav_msgs/Odometry`                     | (Optional) Robot odometry for stationary detection |

### Published Topics

| Topic            | Type                           | Description                     |
| ---------------- | ------------------------------ | ------------------------------- |
| `bias_out_topic` | `geometry_msgs/Vector3Stamped` | Estimated angular velocity bias |

### Parameters

| Parameter           | Type   | Default     | Description                                 |
| ------------------- | ------ | ----------- | ------------------------------------------- |
| `imu_in_topic`      | string | `"imu"`     | Input IMU topic                             |
| `bias_out_topic`    | string | `"bias"`    | Output bias topic                           |
| `cmd_vel_topic`     | string | `"cmd_vel"` | Cmd_vel topic for stationary detection      |
| `odom_topic`        | string | `"odom"`    | Odom topic for stationary detection         |
| `use_cmd_vel`       | bool   | `false`     | Enable cmd_vel for stationary detection     |
| `use_odom`          | bool   | `false`     | Enable odometry for stationary detection    |
| `use_stamped`       | bool   | `false`     | Use `TwistStamped` instead of `Twist`       |
| `stationary_mode`   | string | `"OR"`      | Stationary policy: `"OR"` or `"AND"`        |
| `accumulator_alpha` | double | `0.01`      | EWMA alpha for bias accumulation            |
| `cmd_vel_threshold` | double | `0.001`     | Cmd_vel threshold for stationary detection  |
| `odom_threshold`    | double | `0.001`     | Odometry threshold for stationary detection |
| `estimator_rate_hz` | double | `50.0`      | Bias estimation and publishing frequency    |

---

## 2. BiasRemover Node

**Node Name:** `imu_bias_remover`
**Package:** `perseus_sensors`
**Plugin (Composable):** `imu_processors::BiasRemover`

### Description

Removes the latest bias from the IMU angular velocity and republishes corrected IMU messages. Can throttle output rate.

### Subscribed Topics

| Topic           | Type                           | Description                         |
| --------------- | ------------------------------ | ----------------------------------- |
| `imu_in_topic`  | `sensor_msgs/Imu`              | Raw IMU data                        |
| `bias_in_topic` | `geometry_msgs/Vector3Stamped` | Bias estimated from `BiasEstimator` |

### Published Topics

| Topic           | Type              | Description        |
| --------------- | ----------------- | ------------------ |
| `imu_out_topic` | `sensor_msgs/Imu` | Bias-corrected IMU |

### Parameters

| Parameter           | Type   | Default                | Description                                       |
| ------------------- | ------ | ---------------------- | ------------------------------------------------- |
| `imu_in_topic`      | string | `"imu"`                | Input IMU topic                                   |
| `imu_out_topic`     | string | `"imu_bias_corrected"` | Corrected IMU output topic                        |
| `bias_in_topic`     | string | `"bias"`               | Input bias topic                                  |
| `zero_when_no_bias` | bool   | `false`                | If no bias received, set angular velocity to zero |
| `output_rate_hz`    | double | `50.0`                 | Throttle publishing rate                          |

---

## 3. TopicRemapper Node

**Node Name:** `topic_remapper`
**Package:** `perseus_sensors`

### Description

Dynamically detects the message type of an input topic and republishes it at a lower frequency. Useful for downsampling high-frequency topics such as IMU or LiDAR.

### Subscribed Topics

| Topic         | Type    | Description           |
| ------------- | ------- | --------------------- |
| `input_topic` | dynamic | Source topic to remap |

### Published Topics

| Topic          | Type    | Description                       |
| -------------- | ------- | --------------------------------- |
| `output_topic` | dynamic | Remapped topic at lower frequency |

### Parameters

| Parameter             | Type   | Default      | Description               |
| --------------------- | ------ | ------------ | ------------------------- |
| `input_topic`         | string | -            | Topic to remap (required) |
| `output_topic`        | string | `"remapped"` | Remapped output topic     |
| `reduction_frequency` | double | `10.0`       | Output frequency in Hz    |
