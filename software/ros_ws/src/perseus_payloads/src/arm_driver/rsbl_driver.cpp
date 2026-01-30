#include "arm_driver/rsbl_driver.hpp"

#include <string>

using namespace hi_can;
using namespace hi_can::addressing::post_landing;
using namespace hi_can::addressing::post_landing::arm::control_board;
using namespace hi_can::parameters::post_landing::arm::control_board;

ArmController::ArmController(const rclcpp::NodeOptions& options)
    : Node("arm_controller", options)
{
    try
    {
        _can_interface.emplace(RawCanInterface(this->declare_parameter("can_bus", "can0")));
        _packet_manager.emplace(_can_interface.value());
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(this->get_logger(), "Failed to initialise CAN bus: %s", e.what());
    }

    // Timers
    _packet_timer = this->create_wall_timer(PACKET_HANDLE_MS, std::bind(&ArmController::_handle_can, this));
    _status_timer = this->create_wall_timer(std::chrono::milliseconds(this->_status_message_ms),
                                            std::bind(&ArmController::_publish_status_messages, this));
    _motor_position_timer = this->create_wall_timer(POSITION_PUBLISH_MS,
                                                    std::bind(&ArmController::_publish_motor_positions, this));
    _status_request_timer = this->create_wall_timer(STATUS_REQUEST_MS,
                                                    std::bind(&ArmController::_request_servo_status, this));

    for (const auto& [servo_id, parameter_group] : this->PARAMETER_GROUP_MAP)  // Add RSBL parameter groups
    {
        if (_packet_manager)
        {
            _packet_manager->add_group(*parameter_group);
        }
        _available_servos.push_back(servo_id);
    }

    // Publishers
    _status_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("/arm/rsbl/status", 10);
    _motor_position_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("/arm/rsbl/positions", 10);

    // Subscriber
    _arm_control_subscriber = this->create_subscription<perseus_msgs::msg::ArmControl>(
        "/arm/rsbl/control", 10, std::bind(&ArmController::_handle_arm_control, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Arm Controller initialized - controlling RSBL servos and PWM outputs");
}

void ArmController::_handle_arm_control(const perseus_msgs::msg::ArmControl::SharedPtr msg)
{
    uint8_t acceleration = msg->acceleration;

    if (msg->position.size() >= 2)
    {
        int16_t pos_tilt = static_cast<int16_t>((msg->position[0] * (4096.0 / (2.0 * M_PI))) + 2048.0);
        int16_t pos_pan = static_cast<int16_t>((msg->position[1] * (4096.0 / (2.0 * M_PI))) + 2048.0);

        uint16_t speed_tilt = 0;
        uint16_t speed_pan = 0;

        if (msg->velocity.size() >= 2)
        {
            speed_tilt = static_cast<uint16_t>(std::abs(msg->velocity[0] * (4096.0 / (2.0 * M_PI))));
            speed_pan = static_cast<uint16_t>(std::abs(msg->velocity[1] * (4096.0 / (2.0 * M_PI))));
        }

        // if (_can_interface) {
        //     _can_interface->transmit(Packet(
        //         static_cast<hi_can::addressing::flagged_address_t>(servo_address_t(static_cast<uint8_t>(command_t::WRITE_POS_EX), group::SHOULDER_TILT)),
        //         send_message::write_pos_ex_t{ pos_tilt, speed_tilt, acceleration }));
        // }

        std::this_thread::sleep_for(PACKET_DELAY_MS);

        // if (_can_interface) {
        //     _can_interface->transmit(Packet(
        //         static_cast<hi_can::addressing::flagged_address_t>(servo_address_t(static_cast<uint8_t>(command_t::WRITE_POS_EX), group::SHOULDER_PAN)),
        //         send_message::write_pos_ex_t{ pos_pan, speed_pan, acceleration }));
        // }

        std::this_thread::sleep_for(PACKET_DELAY_MS);
    }

    for (size_t i = 0; i < msg->normalized.size() && i < 2; i++)  // Handle PWM commands (convert normalized 0.0-1.0 to 0-4095)
    {
        group pwm_id = (i == 0) ? group::PWM_1 : group::PWM_2;
        uint16_t pwm_value = static_cast<uint16_t>(msg->normalized[i] * 4095.0);

        // if (_can_interface) {
        //     _can_interface->transmit(Packet(
        //         static_cast<hi_can::addressing::flagged_address_t>(servo_address_t(static_cast<uint8_t>(command_t::SET_PWM_VALUES), pwm_id)),
        //         send_message::set_pwm_t(pwm_value).serialize_data()));
        // }
        std::this_thread::sleep_for(PACKET_DELAY_MS);
    }
}

void ArmController::_publish_status_messages()
{
    std_msgs::msg::Float64MultiArray status_msg;

    for (const auto& servo_id : this->_available_servos)  // Publish RSBL servo status
    {
        auto it = this->PARAMETER_GROUP_MAP.find(servo_id);
        if (it != this->PARAMETER_GROUP_MAP.end())
        {
            const auto& parameter_group = it->second;
            // status_msg.data.emplace_back(static_cast<double>(parameter_group->get_position()));
            // status_msg.data.emplace_back(static_cast<double>(parameter_group->get_speed()));
            // status_msg.data.emplace_back(static_cast<double>(parameter_group->get_load()));
            // status_msg.data.emplace_back(static_cast<double>(parameter_group->get_voltage()));
            // status_msg.data.emplace_back(static_cast<double>(parameter_group->get_temperature()));
            // status_msg.data.emplace_back(static_cast<double>(parameter_group->get_current()));
            // status_msg.data.emplace_back(static_cast<double>(parameter_group->is_moving() ? 1 : 0));
        }
    }

    _status_publisher->publish(status_msg);
}

void ArmController::_publish_motor_positions()
{
    std_msgs::msg::Float64MultiArray position_msg;
    position_msg.data = std::vector<double>(this->PARAMETER_GROUP_MAP.size(), 0.0);

    for (const auto& servo_id : this->_available_servos)
    {
        auto it = this->PARAMETER_GROUP_MAP.find(servo_id);
        if (it != this->PARAMETER_GROUP_MAP.end())
        {
            const auto& parameter_group = it->second;
            // position_msg.data[static_cast<size_t>(servo_id)] = static_cast<double>(parameter_group->get_position());
        }
    }

    _motor_position_publisher->publish(position_msg);
}

void ArmController::_handle_can()
{
    try
    {
        if (_packet_manager)
        {
            _packet_manager->handle();
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000, "CAN Handle failed: %s", e.what());
    }
}

void ArmController::_request_servo_status()
{
    for (const auto& servo_id : this->_available_servos)  // Request RSBL servo status
    {
        // if (_can_interface) {
        //     _can_interface->transmit(Packet(
        //         static_cast<hi_can::addressing::flagged_address_t>(servo_address_t(static_cast<uint8_t>(command_t::READ_STATUS_1), servo_id)),
        //         std::vector<uint8_t>{}));
        // }
        std::this_thread::sleep_for(PACKET_DELAY_MS);

        // if (_can_interface) {
        //     _can_interface->transmit(Packet(
        //         static_cast<hi_can::addressing::flagged_address_t>(servo_address_t(static_cast<uint8_t>(command_t::READ_STATUS_2), servo_id)),
        //         std::vector<uint8_t>{}));
        // }
        std::this_thread::sleep_for(PACKET_DELAY_MS);
    }
}

void ArmController::cleanup()
{
    _packet_manager.reset();
    _can_interface.reset();
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
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Error running Arm Controller: %s", e.what());
        return EXIT_FAILURE;
    }
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
