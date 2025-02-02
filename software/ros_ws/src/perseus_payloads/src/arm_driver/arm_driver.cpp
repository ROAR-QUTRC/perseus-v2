#include "arm_driver/arm_driver.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace st3215;

ArmDriver::ArmDriver(const rclcpp::NodeOptions& options)
    : Node("arm_driver", options)
{
    // Create and configure the servo
    try
    {
        _servoManager = std::make_unique<ServoManager>("/dev/ttyUSB0");  // Servo ID 1

        // Add servo ID 1
        _servoManager->addServo(1);
        auto servo = _servoManager->getServo(1);
        // Configure servo parameters
        servo->setOperatingMode(operating_mode_t::POSITION);
        servo->setTorque(true);
        // servo->setSpeedLimit(500);   // Set moderate speed
        // servo->setAcceleration(50);  // Set moderate acceleration

        RCLCPP_INFO(get_logger(), "Successfully connected to servo ID 1");
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(get_logger(), "Failed to initialize servo: %s", e.what());
        throw;
    }

    // Create joint state publisher
    _joint_state_pub = create_publisher<sensor_msgs::msg::JointState>(
        "joint_states", 10);

    // Create timer for sweep motion (20Hz)
    _sweep_timer = create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&ArmDriver::_sweep_callback, this));

    RCLCPP_INFO(get_logger(), "Arm driver initialized, starting sweep motion");
}

void ArmDriver::_sweep_callback()
{
    try
    {
        // Update position based on direction
        if (_sweep_direction)
        {
            _current_pos += _step_size;
            if (_current_pos >= _max_pos)
            {
                _current_pos = _max_pos;
                _sweep_direction = false;
            }
        }
        else
        {
            _current_pos -= _step_size;
            if (_current_pos <= _min_pos)
            {
                _current_pos = _min_pos;
                _sweep_direction = true;
            }
        }

        // Command the servo to move to the new position
        _servoManager->getServo(1)->setPosition(_current_pos);

        // Get actual position from servo and publish
        // uint16_t actual_pos = _servoManager->getServo(1)->getPosition();
        // _publish_joint_state(actual_pos);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(get_logger(), "Error during sweep: %s", e.what());
    }
}

void ArmDriver::_publish_joint_state(uint16_t position)
{
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = now();
    msg.name.push_back("servo_joint1");

    // Convert servo position (0-1023) to radians (-π to π)
    double position_rad = (static_cast<double>(position) / 1023.0 * 2.0 * M_PI) - M_PI;
    msg.position.push_back(position_rad);

    _joint_state_pub->publish(msg);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<ArmDriver>();
        RCLCPP_INFO(rclcpp::get_logger("main"), "Starting arm driver node...");
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Error running arm driver: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}