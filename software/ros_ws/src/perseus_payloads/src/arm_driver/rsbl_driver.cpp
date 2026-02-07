#include "arm_driver/rsbl_driver.hpp"

#include <string>

using namespace hi_can;
using namespace hi_can::addressing::post_landing;
using namespace hi_can::addressing::post_landing::arm::control_board;
using namespace hi_can::parameters::post_landing::arm::control_board;

RsblDriver::RsblDriver(const rclcpp::NodeOptions& options)
    : Node("rsbl_servo_driver", options) {
    try {
        _can_interface.emplace(RawCanInterface(this->declare_parameter("can_bus", "can0")));
        _packet_manager.emplace(_can_interface.value());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(this->get_logger(), "Failed to initialise CAN bus: %s", e.what());
    }

    // Timers
    _packet_timer = this->create_wall_timer(PACKET_HANDLE_MS, std::bind(&RsblDriver::_handle_can, this));
    _status_timer = this->create_wall_timer(std::chrono::milliseconds(this->_status_message_ms),
        std::bind(&RsblDriver::_publish_status_messages, this));
    _motor_position_timer = this->create_wall_timer(POSITION_PUBLISH_MS,
        std::bind(&RsblDriver::_publish_motor_positions, this));

    for (const auto& [servo_id, parameter_group] : this->PARAMETER_GROUP_MAP)  // Add RSBL parameter groups
    {
        if (_packet_manager) {
            _packet_manager->add_group(*parameter_group);
        }
        _available_servos.push_back(servo_id);
    }

    // Publishers
    _status_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("/arm/rsbl/status", 10);
    _motor_position_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("/arm/rsbl/positions", 100);

    // Subscriber
    _arm_control_subscriber = this->create_subscription<actuator_msgs::msg::Actuators>(
        "/arm/rsbl/control", 10, std::bind(&RsblDriver::_handle_arm_control, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Arm Controller initialized");
}

// Normalised field contains: [time_ms, acceleration]
void RsblDriver::_handle_arm_control(const actuator_msgs::msg::Actuators::SharedPtr msg) {
    uint8_t acceleration = msg->normalized[1];

    if (msg->position.size() >= 2) {
        int16_t pos_tilt = static_cast<int16_t>((msg->position[0] * (4096.0 / (2.0 * M_PI))) + 2048.0);
        int16_t pos_pan = static_cast<int16_t>((msg->position[1] * (4096.0 / (2.0 * M_PI))) + 2048.0);

        uint16_t speed_tilt = 0;
        uint16_t speed_pan = 0;

        if (msg->velocity.size() >= 2) {
            speed_tilt = static_cast<uint16_t>(std::abs(msg->velocity[0] * (4096.0 / (2.0 * M_PI))));
            speed_pan = static_cast<uint16_t>(std::abs(msg->velocity[1] * (4096.0 / (2.0 * M_PI))));
        }

        using namespace hi_can::addressing;

        position_control_t position_control{};

        position_control.position = pos_pan;
        position_control.duration_ms = speed_pan;
        position_control.acceleration = acceleration;
        _can_interface->transmit(Packet{
            static_cast<flagged_address_t>(standard_address_t(this->baseAddress,
                                                              static_cast<uint8_t>(group::SHOULDER_PAN),
                                                              static_cast<uint8_t>(rsbl_parameters::SET_POS_EX))),
            position_control.serialize_data() });

        position_control.position = pos_tilt;
        position_control.duration_ms = speed_tilt;
        position_control.acceleration = acceleration;
        _can_interface->transmit(Packet{
            static_cast<flagged_address_t>(standard_address_t(this->baseAddress,
                                                              static_cast<uint8_t>(group::SHOULDER_TILT),
                                                              static_cast<uint8_t>(rsbl_parameters::SET_POS_EX))),
            position_control.serialize_data() });
    }
}

void RsblDriver::_publish_status_messages() {
    std_msgs::msg::Float64MultiArray status_msg;

    using namespace hi_can::parameters::post_landing::arm::control_board;

    for (const auto& servo_id : this->_available_servos)  // Publish RSBL servo status
    {
        auto it = this->PARAMETER_GROUP_MAP.find(servo_id);
        if (it != this->PARAMETER_GROUP_MAP.end()) {
            const auto& parameter_group = it->second;

            const status_1_t& status1 = parameter_group->get_status_1();
            const status_2_t& status2 = parameter_group->get_status_2();
            status_msg.data.emplace_back(static_cast<double>(servo_id));
            status_msg.data.emplace_back(-1);  // no error code available
            status_msg.data.emplace_back(static_cast<double>(status1.position));
            status_msg.data.emplace_back(static_cast<double>(status1.speed));
            status_msg.data.emplace_back(static_cast<double>(status2.temperature));
            status_msg.data.emplace_back(static_cast<double>(status2.voltage));
            status_msg.data.emplace_back(static_cast<double>(status2.current));
            status_msg.data.emplace_back(static_cast<double>(status1.load));
        }
    }

    _status_publisher->publish(status_msg);
}

void RsblDriver::_publish_motor_positions() {
    std_msgs::msg::Float64MultiArray position_msg;

    for (const auto& servo_id : this->_available_servos) {
        auto it = this->PARAMETER_GROUP_MAP.find(servo_id);
        if (it != this->PARAMETER_GROUP_MAP.end()) {
            const auto& parameter_group = it->second;
            position_msg.data.emplace_back(static_cast<double>(parameter_group->get_position()));
        }
    }

    _motor_position_publisher->publish(position_msg);
}

void RsblDriver::_handle_can() {
    try {
        if (_packet_manager) {
            _packet_manager->handle();
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000, "CAN Handle failed: %s", e.what());
    }
}

void RsblDriver::cleanup() {
    _packet_manager.reset();
    _can_interface.reset();
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<RsblDriver>();
        RCLCPP_INFO(rclcpp::get_logger("main"), "Starting RSBL Servo driver node");
        rclcpp::spin(node);
        node->cleanup();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Error running RSBL Servo driver: %s", e.what());
        return EXIT_FAILURE;
    }
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
