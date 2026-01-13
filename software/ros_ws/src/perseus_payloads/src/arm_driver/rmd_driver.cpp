#include "arm_driver/rmd_driver.hpp"

#include <string>

// Topics and services:
// /arm
//     /rmd
//         /control (std_msgs/Float64MultiArray) - target positions for servos (subscriber)
//         /status (std_msgs/Float64MultiArray) - status messages from servos (publisher)
//         /positions (std_msgs/Float64MultiArray) - current positions of servos (publisher)
//         /enable_debug_stats (std_srvs/SetBool) - enable/disable debug status messages (service)
//         /set_id (perseus_msgs/TriggerDevice) - set motor ID service (service)
//         /set_zero_position (perseus_msgs/TriggerDevice) - set current position as zero position (service)
//         /restart_motor (perseus_msgs/TriggerDevice) - restart motor service (service)
//         /get_can_ids (perseus_msgs/RequestInt8Array) - get list of active RMD CAN IDs (service)

using namespace hi_can;
using namespace hi_can::addressing::post_landing;
using namespace hi_can::addressing::post_landing::servo::rmd;
using namespace hi_can::parameters::post_landing::servo::rmd;

RmdDriver::RmdDriver(const rclcpp::NodeOptions& options)
    : Node("rmd_servo_driver", options)
{
    try
    {
        _can_interface.emplace(RawCanInterface(this->declare_parameter("can_bus", "can0")));
        _packet_manager.emplace(_can_interface.value());
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL(this->get_logger(), "Failed to initialise CAN bus: %s", e.what());
        return;
    }

    // Add RMD parameter groups to packet manager
    for (const auto& [motor_id, parameter_group] : this->PARAMETER_GROUP_MAP)
    {
        _packet_manager->add_group(*parameter_group);
    }
    _packet_timer = this->create_wall_timer(PACKET_HANDLE_MS, std::bind(&RmdDriver::_handle_can, this));

    // Initialise all motors on startup with desired functions
    // Stop all motors - motors will lock up and won't flop around
    _can_interface->transmit(Packet{
        servo_address_t{message_type::MULTI_MOTOR_SEND},
        send_message::action_command_t(send_message::action_command_t::command_id_t::STOP).serialize_data()});
    std::this_thread::sleep_for(PACKET_DELAY_MS);
    this->_enable_status_messages(false);
    _motor_target_positions_subscriber = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/arm/rmd/control", 10, std::bind(&RmdDriver::_handle_position_control, this, std::placeholders::_1));

    // Setup motor feedback handling
    _status_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("/arm/rmd/status", 10);
    _status_timer = this->create_wall_timer(std::chrono::milliseconds(this->_status_message_ms), std::bind(&RmdDriver::_publish_status_messages, this));
    _enable_debug_stats_service = this->create_service<std_srvs::srv::SetBool>(
        "/arm/rmd/enable_debug_stats",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response)
        {
            this->_enable_status_messages(request->data);
            response->success = this->_status_message_ms == 50;
        });

    // Setup config Services
    _set_motor_id_service = this->create_service<perseus_msgs::srv::TriggerDevice>("/arm/rmd/set_id", std::bind(&RmdDriver::_set_motor_id, this, std::placeholders::_1, std::placeholders::_2));
    _set_zero_position_service = this->create_service<perseus_msgs::srv::TriggerDevice>("/arm/rmd/set_zero_position", std::bind(&RmdDriver::_set_zero_position, this, std::placeholders::_1, std::placeholders::_2));
    _restart_motor_service = this->create_service<perseus_msgs::srv::TriggerDevice>("/arm/rmd/restart_motor", std::bind(&RmdDriver::_restart_motor, this, std::placeholders::_1, std::placeholders::_2));
    _get_can_ids_service = this->create_service<perseus_msgs::srv::RequestInt8Array>("/arm/rmd/get_can_ids", std::bind(&RmdDriver::_get_rmd_can_ids, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "RMD servo driver node initialised");
}

void RmdDriver::_enable_status_messages(bool debug_mode)
{
    // Update speed if in debug mode
    this->_status_message_ms = debug_mode ? 50 : 5000;
    if (this->_status_timer)
    {
        this->_status_timer->cancel();
        this->_status_timer = this->create_wall_timer(std::chrono::milliseconds(this->_status_message_ms), std::bind(&RmdDriver::_publish_status_messages, this));
    }
    // Set motors to transmit error messages when there is an error
    _can_interface->transmit(Packet{
        servo_address_t{message_type::MULTI_MOTOR_SEND},
        send_message::function_control_t(send_message::function_control_t::function_index_t::ERROR_TRANSMISSION_ENABLE, 1).serialize_data()});
    std::this_thread::sleep_for(PACKET_DELAY_MS);

    // Set motors to transmit all three status messages every second
    _can_interface->transmit(Packet{
        servo_address_t{message_type::MULTI_MOTOR_SEND},
        send_message::active_reply_t(send_message::active_reply_t::reply_t::STATUS_1, true, this->_status_message_ms).serialize_data()});
    std::this_thread::sleep_for(PACKET_DELAY_MS);
    _can_interface->transmit(Packet{
        servo_address_t{message_type::MULTI_MOTOR_SEND},
        send_message::active_reply_t(send_message::active_reply_t::reply_t::STATUS_2, true, this->_status_message_ms).serialize_data()});
    std::this_thread::sleep_for(PACKET_DELAY_MS);
    _can_interface->transmit(Packet{
        servo_address_t{message_type::MULTI_MOTOR_SEND},
        send_message::active_reply_t(send_message::active_reply_t::reply_t::STATUS_3, true, this->_status_message_ms).serialize_data()});
    std::this_thread::sleep_for(PACKET_DELAY_MS);  // small delay otherwise messages are ignored by servos

    // Send position feedback every position publish interval
    // _can_interface->transmit(Packet{
    //     servo_address_t{message_type::MULTI_MOTOR_SEND},
    //     send_message::active_reply_t(send_message::active_reply_t::reply_t::MULTI_TURN_POSITION, true, this->POSITION_PUBLISH_MS).serialize_data()});
    // std::this_thread::sleep_for(PACKET_DELAY_MS);
}

void RmdDriver::_handle_position_control(std_msgs::msg::Float64MultiArray servo_control)
{
    // const double target_time_ms = servo_control.normalized[0];
    // const double max_angle = *std::max_element(servo_control.position.begin(), servo_control.position.end());
    // uint16_t degrees_per_second = max_angle / (target_time_ms / 1000.0);
    // degrees_per_second = std::clamp(degrees_per_second, (uint16_t)0, RMD_SPEED_LIMIT);

    for (const auto& motor_id : this->_get_online_servos())
    {
        const uint8_t index = static_cast<uint8_t>(motor_id) - 1;
        const double target_position = servo_control.data[index];
        _can_interface->transmit(Packet{
            servo_address_t(message_type::SEND, motor_id),
            send_message::position_t(
                send_message::position_t::position_command_t::ABSOLUTE,
                // degrees_per_second,
                RMD_SPEED_LIMIT * 0.5,
                target_position)
                .serialize_data()});
    }
}

void RmdDriver::_publish_status_messages()
{
    std_msgs::msg::Float64MultiArray status_msg;

    for (const auto& motor_id : this->_get_online_servos())
    {
        auto it = this->PARAMETER_GROUP_MAP.find(motor_id);
        if (it != this->PARAMETER_GROUP_MAP.end())
        {
            const auto& parameter_group = it->second;
            status_msg.data.emplace_back(static_cast<double>(motor_id));
            status_msg.data.emplace_back(static_cast<uint16_t>(parameter_group->get_error_status()));
            status_msg.data.emplace_back(static_cast<double>(parameter_group->get_temp()));
            status_msg.data.emplace_back(parameter_group->get_voltage());
            status_msg.data.emplace_back(parameter_group->get_torque_current());
            status_msg.data.emplace_back(static_cast<double>(parameter_group->get_speed()));
            status_msg.data.emplace_back(static_cast<double>(parameter_group->get_angle()));
            status_msg.data.emplace_back(parameter_group->get_phase_a_current());
            status_msg.data.emplace_back(parameter_group->get_phase_b_current());
            status_msg.data.emplace_back(parameter_group->get_phase_c_current());
        }
    }

    _status_publisher->publish(status_msg);
}

void RmdDriver::_handle_can()
{
    try
    {
        _packet_manager->handle();
    }
    catch (const std::exception& e)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000, "CAN Handle failed: %s", e.what());
    }

    for (const auto& [motor_id, parameter_group] : this->PARAMETER_GROUP_MAP)
    {
        std::vector<std::string> errors = parameter_group->check_errors();
        if (errors.size())
        {
            for (std::string error : errors)
            {
                RCLCPP_ERROR(this->get_logger(), "Motor ID %d error: %s", static_cast<uint8_t>(motor_id), error.c_str());
            }
        }
    }
}

std::vector<motor_id_t> RmdDriver::_get_online_servos()
{
    std::vector<motor_id_t> online_servos;
    for (const auto& [motor_id, parameter_group] : this->PARAMETER_GROUP_MAP)
    {
        if (parameter_group->is_online())
        {
            online_servos.emplace_back(motor_id);
        }
    }
    return online_servos;
}

#pragma region CONFIG_SERVICES

void RmdDriver::_set_motor_id(const std::shared_ptr<perseus_msgs::srv::TriggerDevice::Request> request, std::shared_ptr<perseus_msgs::srv::TriggerDevice::Response> response)
{
    using function = hi_can::parameters::post_landing::servo::rmd::send_message::function_control_t::function_index_t;

    const motor_id_t current_id = static_cast<motor_id_t>(request->id);

    _can_interface->transmit(Packet{
        servo_address_t(message_type::SEND, current_id),
        send_message::function_control_t(function::SET_CANID, static_cast<uint32_t>(request->data)).serialize_data()});
    std::this_thread::sleep_for(PACKET_DELAY_MS);
    // Since the ID is currently in an undefined state, MULTI_MOTOR_SEND must be used to ensure the motor is reset
    _can_interface->transmit(Packet{
        servo_address_t(message_type::MULTI_MOTOR_SEND),
        send_message::action_command_t(send_message::action_command_t::command_id_t::RESET).serialize_data()});
    std::this_thread::sleep_for(PACKET_DELAY_MS);

    PARAMETER_GROUP_MAP.at(current_id)->set_online(false);  // mark old ID as offline

    this->_enable_status_messages(this->_status_message_ms < 500);

    response->success = true;
}

void RmdDriver::_set_zero_position(const std::shared_ptr<perseus_msgs::srv::TriggerDevice::Request> request, std::shared_ptr<perseus_msgs::srv::TriggerDevice::Response> response)
{
    const motor_id_t motor_id = static_cast<motor_id_t>(request->id);

    _can_interface->transmit(Packet{
        servo_address_t(message_type::SEND, motor_id),
        send_message::zero_offset_t(request->trigger, static_cast<uint32_t>(request->data)).serialize_data()});
    std::this_thread::sleep_for(PACKET_DELAY_MS);
    _can_interface->transmit(Packet{
        servo_address_t(message_type::SEND, motor_id),
        send_message::action_command_t(send_message::action_command_t::command_id_t::RESET).serialize_data()});
    std::this_thread::sleep_for(PACKET_DELAY_MS);
    response->success = true;
}

void RmdDriver::_restart_motor(const std::shared_ptr<perseus_msgs::srv::TriggerDevice::Request> request, std::shared_ptr<perseus_msgs::srv::TriggerDevice::Response> response)
{
    _can_interface->transmit(Packet{
        servo_address_t(message_type::SEND, static_cast<motor_id_t>(request->id)),
        send_message::action_command_t(send_message::action_command_t::command_id_t::SHUTDOWN).serialize_data()});
    response->success = true;
}

void RmdDriver::_get_rmd_can_ids(const std::shared_ptr<perseus_msgs::srv::RequestInt8Array::Request> request, std::shared_ptr<perseus_msgs::srv::RequestInt8Array::Response> response)
{
    (void)request;  // request is empty

    response->data.clear();
    for (const auto& motor_id : this->_get_online_servos())
    {
        response->data.emplace_back(static_cast<uint8_t>(motor_id));
    }

    // Not ideal but resetting all the online servos to offline will ensure disconnected servos are removed.
    for (const auto& [motor_id, parameter_group] : this->PARAMETER_GROUP_MAP)
    {
        parameter_group->set_online(false);
    }
}

#pragma endregion CONFIG_SERVICES

void RmdDriver::cleanup()
{
    _packet_manager.reset();
    _can_interface.reset();
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<RmdDriver>();
        RCLCPP_INFO(rclcpp::get_logger("main"), "Starting RMD Servo driver node");
        rclcpp::spin(node);
        node->cleanup();
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Error running RMD Servo driver: %s", e.what());
        return EXIT_FAILURE;
    }
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}