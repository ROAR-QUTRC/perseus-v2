# Topic Remapper Node

A ROS 2 node that dynamically detects the message type of a topic and republishes it at a lower frequency.

## Overview

The Topic Remapper node subscribes to any ROS 2 topic, automatically detects its message type, and republishes the messages to a different topic at a configurable reduced frequency. This is useful for:

- Reducing bandwidth and computational load of high-frequency sensors
- Downsampling topics without needing to know the message type at compile time
- Creating lower-frequency copies of real-time data streams

## Features

- **Dynamic Type Detection**: Automatically detects the message type of the input topic
- **Serialization-based**: Uses ROS 2's serialization to handle any message type without compile-time knowledge
- **Configurable Parameters**: Control input topic, output topic, and target frequency
- **Debug Logging**: Comprehensive logging for monitoring and troubleshooting

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `input_topic` | string | (required) | The name of the input topic to subscribe to |
| `output_topic` | string | "remapped" | The name of the output topic to publish to |
| `reduction_frequency` | double | 10.0 | Target output frequency in Hz (must be positive) |

## Usage

### Basic Usage

```bash
ros2 run perseus_sensors topic_remapper --ros-args -p input_topic:=/scan -p output_topic:=/scan_remapped -p reduction_frequency:=5.0
```

### Using Launch File

```bash
ros2 launch perseus_sensors topic_remapper.launch.xml input_topic:=/scan output_topic:=/scan_remapped reduction_frequency:=5.0
```

### Example with Default Parameters

```bash
ros2 launch perseus_sensors topic_remapper.launch.xml
```

This will:
- Subscribe to `/scan` topic
- Republish to `/scan_remapped` topic
- Output at 10 Hz frequency

## How It Works

1. **Initialization**: The node loads ROS parameters and validates them
2. **Type Detection**: When the node starts, it queries the ROS 2 graph for topic names and types
3. **Subscription**: A generic subscription is created using `rclcpp::SerializedMessage`
4. **Frequency Reduction**: Incoming messages are timestamped, and only messages that arrive after the calculated publish interval are republished
5. **Publishing**: Detected messages that pass the frequency check are published to the output topic

### Frequency Reduction Logic

The node maintains a timestamp of the last published message. When a new message arrives:

1. Calculate time elapsed since the last publish
2. If elapsed time ≥ `1 / reduction_frequency`, publish the message and update timestamp
3. Otherwise, skip the message

For example, with `reduction_frequency=10.0`:
- Publish interval = 1000ms / 10 = 100ms
- Messages arriving within 100ms of the last publish are skipped
- Only messages arriving ≥100ms after the last publish are republished

## Example Scenarios

### Scenario 1: Reduce LiDAR Scan Rate
```bash
ros2 launch perseus_sensors topic_remapper.launch.xml \
  input_topic:=/lidar/scan \
  output_topic:=/lidar/scan_10hz \
  reduction_frequency:=10.0
```

### Scenario 2: Reduce IMU Frequency from 100Hz to 20Hz
```bash
ros2 launch perseus_sensors topic_remapper.launch.xml \
  input_topic:=/imu/data \
  output_topic:=/imu/data_low_freq \
  reduction_frequency:=20.0
```

### Scenario 3: Reduce Camera Image Stream
```bash
ros2 launch perseus_sensors topic_remapper.launch.xml \
  input_topic:=/camera/image_raw \
  output_topic:=/camera/image_raw_reduced \
  reduction_frequency:=5.0
```

## Limitations

- The node detects the topic type when it starts. If no publishers are active when the node launches, it will still attempt to subscribe (topics can connect later in ROS 2)
- Frequency reduction is approximate due to system scheduling and timing precision
- For frame-based filtering (e.g., only publish every Nth message regardless of timing), use a different approach

## Implementation Details

### Type Detection

The node uses the ROS 2 node graph API (`get_topic_names_and_types()`) to discover what message type is being published on the input topic. This allows it to:
- Work with any message type without compile-time dependencies
- Handle custom message types automatically

### Serialization

The node uses `rclcpp::SerializedMessage` for the subscription and publication, which:
- Avoids deserialization/reserialization overhead
- Works with any message type
- Preserves message data integrity

## Troubleshooting

### Node starts but no messages are published

1. Check that the input topic exists: `ros2 topic list`
2. Check that the input topic has an active publisher: `ros2 topic info /input_topic`
3. Verify parameters: `ros2 param list /topic_remapper`
4. Enable debug logging: `ros2 run perseus_sensors topic_remapper --ros-args -p input_topic:=/scan --log-level debug`

### Output frequency is not as expected

The frequency reduction is approximate and depends on:
- The actual frequency of incoming messages
- System scheduling and timing precision
- Other processes running on the system

If the input topic publishes less frequently than the target frequency, the output will be limited by the input frequency.

### Message type not detected

If the topic type shows as empty in logs:
- The topic may not have any publishers when the node started
- Wait for publishers to connect, or restart the node after publishers are running
- The node will still function, it just won't have type information for logging

## Performance Considerations

- **CPU Usage**: Minimal. The node mainly performs timing checks and message copying.
- **Memory**: Uses a small, bounded amount of memory for buffering.
- **Latency**: Adds minimal latency (a few milliseconds at most).

## Related Nodes

- `topic_relay` - Different approach for topic remapping
- Standard ROS 2 utilities: `ros2 topic` command with various sub-commands
