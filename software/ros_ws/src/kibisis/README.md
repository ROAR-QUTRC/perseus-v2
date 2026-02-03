# Kibisis Robot

Kibisis is a two-wheel differential drive robot designed for the Raspberry Pi platform. It uses a GPIO-based motor control board for wheel actuation, similar in concept to TurtleBot3 or iRobot Roomba robots.

## Robot Specifications

| Parameter            | Value                                 |
| -------------------- | ------------------------------------- |
| Drive type           | Two-wheel differential drive          |
| Wheel diameter       | 20 cm (0.2 m)                         |
| Wheel separation     | 30 cm (0.3 m)                         |
| Max linear velocity  | 1.0 m/s                               |
| Max angular velocity | 3.0 rad/s                             |
| Camera               | Forward-facing, 4 cm above wheel axle |
| Sensors              | Camera only (no lidar, no IMU)        |

## Package Structure

The Kibisis robot consists of three ROS2 packages:

```
kibisis/                    # Main bringup package
├── launch/                 # Launch files
├── config/                 # Controller configuration
└── urdf/                   # Main URDF entry point

kibisis_description/        # Robot description
├── urdf/                   # URDF/Xacro files
└── ros2_control/           # Hardware interface config

kibisis_hardware/           # Hardware interface plugin
├── include/                # Header files
└── src/                    # Implementation
```

## Hardware Requirements

### GPIO Motor Controller

Kibisis requires a motor control board that interfaces with the Raspberry Pi's GPIO pins. Unlike robots that use serial-based servo controllers (like Dynamixel), Kibisis uses direct GPIO control for motor actuation.

**Typical motor control board features:**

- H-bridge motor drivers (e.g., L298N, TB6612FNG, or similar)
- PWM input for speed control
- Direction control pins
- Quadrature encoder inputs for position feedback

### Default GPIO Pin Configuration

The default pin assignments in `kibisis_description/ros2_control/kibisis.ros2_control.xacro`:

| Function                    | Left Wheel | Right Wheel |
| --------------------------- | ---------- | ----------- |
| Motor Pin A (PWM/Direction) | GPIO 17    | GPIO 22     |
| Motor Pin B (Direction)     | GPIO 27    | GPIO 23     |
| Encoder Pin A               | GPIO 5     | GPIO 12     |
| Encoder Pin B               | GPIO 6     | GPIO 13     |

**Note:** These pins should be configured to match your specific motor control board wiring.

## Usage

### Launch with Mock Hardware (Testing)

```bash
ros2 launch kibisis kibisis.launch.py use_mock_hardware:=True
```

### Launch with Real Hardware

```bash
ros2 launch kibisis kibisis.launch.py
```

### Teleoperation with Xbox Controller

To teleoperate Kibisis using the generic controller (Xbox, 8BitDo, or Logitech):

**Terminal 1 - Launch Kibisis with joy_vel topic:**
```bash
ros2 launch kibisis kibisis.launch.py use_mock_hardware:=True cmd_vel_topic:=/joy_vel
```

**Terminal 2 - Launch the controller:**
```bash
nix run .#generic_controller
```

The `cmd_vel_topic:=/joy_vel` parameter configures the diff_drive_controller to accept
`TwistStamped` messages from the generic_controller on the `/joy_vel` topic.

### Send Velocity Commands Manually

```bash
# Move forward at 0.2 m/s (using TwistStamped for compatibility)
ros2 topic pub /joy_vel geometry_msgs/msg/TwistStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, twist: {linear: {x: 0.2}, angular: {z: 0.0}}}"

# Or use plain Twist on /cmd_vel (when not using generic_controller)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

## Implementing GPIO Motor Control

The hardware interface in `kibisis_hardware` contains placeholder implementations that need to be completed for real hardware operation.

### Files to Modify

1. **`kibisis_hardware/src/kibisis_system.cpp`**
   - `init_gpio()` - Initialize GPIO pins for motor control
   - `cleanup_gpio()` - Release GPIO resources
   - `set_motor_velocity()` - Send PWM signals to motor controller
   - `read_encoder()` - Read quadrature encoder position

### GPIO Library Options

Choose a GPIO library based on your Raspberry Pi setup:

| Library                                                             | Description                               | Installation                    |
| ------------------------------------------------------------------- | ----------------------------------------- | ------------------------------- |
| [pigpio](http://abyz.me.uk/rpi/pigpio/)                             | Feature-rich, supports PWM and interrupts | `sudo apt install pigpio`       |
| [lgpio](https://github.com/joan2937/lg)                             | Lightweight, newer alternative            | `sudo apt install lg`           |
| [gpiod](https://git.kernel.org/pub/scm/libs/libgpiod/libgpiod.git/) | Linux kernel GPIO interface               | `sudo apt install libgpiod-dev` |

### Example Implementation (pigpio)

```cpp
#include <pigpio.h>

bool KibisisSystemHardware::init_gpio()
{
    if (gpioInitialise() < 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("KibisisSystemHardware"),
                     "Failed to initialize pigpio");
        return false;
    }

    for (const auto& joint : joint_configs_)
    {
        // Set motor pins as outputs
        gpioSetMode(joint.gpio_pin_a, PI_OUTPUT);
        gpioSetMode(joint.gpio_pin_b, PI_OUTPUT);

        // Set encoder pins as inputs with pull-up
        gpioSetMode(joint.encoder_pin_a, PI_INPUT);
        gpioSetMode(joint.encoder_pin_b, PI_INPUT);
        gpioSetPullUpDown(joint.encoder_pin_a, PI_PUD_UP);
        gpioSetPullUpDown(joint.encoder_pin_b, PI_PUD_UP);
    }

    return true;
}

void KibisisSystemHardware::set_motor_velocity(size_t joint_index, double velocity)
{
    const auto& joint = joint_configs_[joint_index];

    // Convert rad/s to PWM duty cycle (0-255)
    // Adjust scaling factor based on your motor characteristics
    int pwm = static_cast<int>(std::abs(velocity) * 50.0);  // Example scaling
    pwm = std::clamp(pwm, 0, 255);

    if (velocity >= 0)
    {
        // Forward direction
        gpioPWM(joint.gpio_pin_a, pwm);
        gpioWrite(joint.gpio_pin_b, 0);
    }
    else
    {
        // Reverse direction
        gpioWrite(joint.gpio_pin_a, 0);
        gpioPWM(joint.gpio_pin_b, pwm);
    }
}
```

### Adding pigpio to CMakeLists.txt

If using pigpio, update `kibisis_hardware/CMakeLists.txt`:

```cmake
# Find pigpio library
find_library(PIGPIO_LIBRARY pigpio)

target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC
    # ... existing libraries ...
    ${PIGPIO_LIBRARY}
)
```

And add to `package.xml`:

```xml
<depend>pigpio</depend>
```

## Topics

| Topic                              | Type                     | Description                                |
| ---------------------------------- | ------------------------ | ------------------------------------------ |
| `/cmd_vel`                         | `geometry_msgs/Twist`    | Velocity commands                          |
| `/diff_drive_base_controller/odom` | `nav_msgs/Odometry`      | Odometry output                            |
| `/joint_states`                    | `sensor_msgs/JointState` | Joint positions/velocities                 |
| `/tf`                              | `tf2_msgs/TFMessage`     | Transform tree                             |
| `/camera/image_raw`                | `sensor_msgs/Image`      | Camera images (when camera driver running) |

## Troubleshooting

### Permission Denied for GPIO

Run with sudo or add user to gpio group:

```bash
sudo usermod -aG gpio $USER
# Log out and back in
```

### Controllers Not Loading

Check that the hardware interface is properly activated:

```bash
ros2 control list_hardware_interfaces
ros2 control list_controllers
```

### Motor Not Responding

1. Verify GPIO pin connections match configuration
2. Check motor power supply
3. Test GPIO pins directly with `gpiotest` or similar tool
