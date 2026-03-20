#include "arm_driver/end_effector.hpp"

EndEffector::EndEffector(const rclcpp::NodeOptions& options)
    : Node("end_effector", options)
{
    RCLCPP_INFO(this->get_logger(), "Initializing End Effector...");

    try
    {
        _can_interface.emplace(hi_can::RawCanInterface(this->declare_parameter("can_bus", "can0")));
        _packet_manager.emplace(_can_interface.value());
        _packet_manager->add_group(*_pwm_parameter_group);
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(this->get_logger(), "Failed to initialise CAN bus: %s", e.what());
        return;
    }

    this->_packet_timer = this->create_wall_timer(
        PACKET_HANDLE_MS, std::bind(&EndEffector::_handle_can_packets, this));

    this->_write = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
        "/arm/end_effector/write", 10, std::bind(&EndEffector::_handle_write, this, std::placeholders::_1));
    this->_read = this->create_publisher<std_msgs::msg::UInt16MultiArray>(
        "/arm/end_effector/read", 10);
    this->_timer = this->create_wall_timer(
        PUBLISH_TIMER_MS, std::bind(&EndEffector::_handle_read, this));

    RCLCPP_INFO(this->get_logger(), "End Effector initialized");
}

void EndEffector::cleanup()
{
    RCLCPP_INFO(this->get_logger(), "Cleaning up End Effector...");
    this->_timer->cancel();
    RCLCPP_INFO(this->get_logger(), "End Effector cleaned up");
}

void EndEffector::_handle_write(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)
{
    using namespace hi_can;
    using namespace hi_can::addressing::post_landing::arm::control_board;
    using namespace hi_can::parameters::post_landing::arm::control_board;

    addressing::standard_address_t address{
        addressing::post_landing::SYSTEM_ID,
        addressing::post_landing::arm::SUBSYSTEM_ID,
        DEVICE_ID};
    address.parameter = static_cast<uint8_t>(pwm_parameters::SET_PWM);
    pwm_t pwm_command{};

    RCLCPP_INFO(this->get_logger(), "Received write command with %zu data bytes", msg->data.size());

    uint8_t index = 0;
    for (const auto& group : {pwm_group::PWM_1, pwm_group::PWM_2, pwm_group::PWM_3, pwm_group::PWM_4})
    {
        pwm_command.value = msg->data[index++];  // Expecting a single uint16 value for PWM command
        address.device = static_cast<uint8_t>(group);
        _can_interface->transmit(Packet{
            static_cast<hi_can::addressing::flagged_address_t>(address),
            pwm_command.serialize_data()});
    }
}

void EndEffector::_handle_read()
{
    std_msgs::msg::UInt16MultiArray msg;
    msg.data.push_back(static_cast<uint16_t>(this->_pwm_parameter_group->get_pwm_1().value));
    msg.data.push_back(static_cast<uint16_t>(this->_pwm_parameter_group->get_pwm_2().value));
    msg.data.push_back(static_cast<uint16_t>(this->_pwm_parameter_group->get_pwm_3().value));
    msg.data.push_back(static_cast<uint16_t>(this->_pwm_parameter_group->get_pwm_4().value));
    _read->publish(msg);
}

void EndEffector::_handle_can_packets()
{
    try
    {
        _packet_manager->handle();
    }
    catch (const std::exception& e)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000, "CAN Handle failed: %s", e.what());
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<EndEffector>();
        RCLCPP_INFO(rclcpp::get_logger("main"), "Starting End Effector node");
        rclcpp::spin(node);
        node->cleanup();
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "Unhandled exception in End Effector node: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}