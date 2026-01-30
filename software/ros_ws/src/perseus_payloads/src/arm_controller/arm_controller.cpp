#include "arm_controller/arm_controller.hpp"

ArmController::ArmController(const rclcpp::NodeOptions& options)
    : Node("arm_controller", options)
{
    RCLCPP_INFO(this->get_logger(), "Initializing Arm Controller...");

    // Initialise ROS2 communications
    // ARM
    _arm_control_subscriber = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/arm/control", 10, std::bind(&ArmController::_handle_arm_control, this, std::placeholders::_1));
    _arm_status_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/arm/status", 10);
    _arm_position_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/arm/positions", 10);
    _position_timer = this->create_wall_timer(
        PUBLISH_TIMER_MS, std::bind(&ArmController::_publish_arm_position, this));
    // RMD
    _rmd_control_publisher = this->create_publisher<actuator_msgs::msg::Actuators>(
        "/arm/rmd/control", 10);
    _rmd_status_subscriber = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/arm/rmd/status", 10,
        std::bind(&ArmController::_receive_rmd_status, this, std::placeholders::_1));
    _rmd_position_subscriber = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/arm/rmd/positions", 10,
        std::bind(&ArmController::_receive_rmd_positions, this, std::placeholders::_1));
    // RSBL
    _rsbl_control_publisher = this->create_publisher<actuator_msgs::msg::Actuators>(
        "/arm/rsbl/control", 10);
    _rsbl_status_subscriber = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/arm/rsbl/status", 10,
        std::bind(&ArmController::_receive_rsbl_status, this, std::placeholders::_1));
    _rsbl_position_subscriber = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/arm/rsbl/positions", 10,
        std::bind(&ArmController::_receive_rsbl_positions, this, std::placeholders::_1));

    // Initialize current arm positions
    _current_arm_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    RCLCPP_INFO(this->get_logger(), "Arm Controller initialized");
}

void ArmController::_publish_arm_status()
{
    auto status_msg = std_msgs::msg::Float64MultiArray();

    // [motor_id, error_code, temperature, voltage, current, rpm, angle, phase_<a,b,c>_current]
    for (size_t i = 0; i < _current_arm_positions.size(); ++i)
    {
        status_msg.data.push_back(i + 1);                      // Motor ID
        status_msg.data.push_back(0.0);                        // Error
        status_msg.data.push_back(0.0);                        // Temperature
        status_msg.data.push_back(0.0);                        // Voltage
        status_msg.data.push_back(0.0);                        // Current
        status_msg.data.push_back(0.0);                        // RPM
        status_msg.data.push_back(_current_arm_positions[i]);  // Angle
        status_msg.data.push_back(0.0);                        // Phase A current
        status_msg.data.push_back(0.0);                        // Phase B current
        status_msg.data.push_back(0.0);                        // Phase C current
    }
    _arm_status_publisher->publish(status_msg);
}

void ArmController::_publish_arm_position()
{
    auto position_msg = std_msgs::msg::Float64MultiArray();
    position_msg.data = _current_arm_positions;
    _arm_position_publisher->publish(position_msg);
}

void ArmController::_receive_rmd_positions(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    // Update current arm positions with RMD servo data
    for (size_t i = 0; i < msg->data.size() && i < _current_arm_positions.size(); ++i)
    {
        // Should be positions 0 to 2
        _current_arm_positions[i] = msg->data[i];
    }
    // Publish updated arm positions
    this->_publish_arm_status();
}

void ArmController::_receive_rsbl_positions(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    // Update current arm positions with RSBL servo data
    for (size_t i = 0; i < msg->data.size() && (i + 3) < _current_arm_positions.size(); ++i)
    {
        // Should be positions 3 to 4
        _current_arm_positions[i + 3] = msg->data[i];
    }
    this->_publish_arm_status();
}

void ArmController::_receive_rmd_status(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    // Handle RMD status message if needed
    RCLCPP_DEBUG(this->get_logger(), "Received RMD status message with %zu data points", msg->data.size());
}

void ArmController::_receive_rsbl_status(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    // Handle RSBL status message if needed
    RCLCPP_DEBUG(this->get_logger(), "Received RSBL status message with %zu data points", msg->data.size());
}

void ArmController::_handle_arm_control(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (msg->data.size() < 5)
    {
        RCLCPP_WARN(this->get_logger(), "Received arm control message with insufficient positions");
        return;
    }

    double target_ms = 3000.0;  // Default time to reach target in ms
    std::vector<double> velocities;

    // calculate velocities
    for (size_t i = 0; i < this->_current_arm_positions.size(); i++)
    {
        double position_diff = std::abs(msg->data[i] - this->_current_arm_positions[i]);
        double velocity = (position_diff / target_ms) * 1000.0;  // in degrees per second
        velocities.push_back(velocity);
    }

    // Prepare RMD control message
    actuator_msgs::msg::Actuators rmd_msg;
    rmd_msg.position = {msg->data[0], msg->data[1], msg->data[2]};
    constexpr double VEL = UINT16_MAX * 0.5;
    rmd_msg.velocity = {VEL, VEL, VEL};
    // rmd_msg.velocity = {velocities[0], velocities[1], velocities[2]};
    _rmd_control_publisher->publish(rmd_msg);

    // Prepare RSBL control message
    actuator_msgs::msg::Actuators rsbl_msg;
    rsbl_msg.position = {msg->data[3], msg->data[4]};
    rsbl_msg.velocity = {velocities[3], velocities[4]};
    _rsbl_control_publisher->publish(rsbl_msg);

    RCLCPP_INFO(this->get_logger(), "Published: RMD positions [%.2f, %.2f, %.2f] with velocities [%.2f, %.2f, %.2f]; RSBL positions [%.2f, %.2f] with velocities [%.2f, %.2f]",
                rmd_msg.position[0], rmd_msg.position[1], rmd_msg.position[2],
                rmd_msg.velocity[0], rmd_msg.velocity[1], rmd_msg.velocity[2],
                rsbl_msg.position[0], rsbl_msg.position[1],
                rsbl_msg.velocity[0], rsbl_msg.velocity[1]);
}

void ArmController::cleanup()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down Arm Controller...");
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<ArmController>();
        RCLCPP_INFO(rclcpp::get_logger("main"), "Starting Arm Controller node");
        rclcpp::spin(node);
        node->cleanup();
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "Unhandled exception in Arm Controller: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}