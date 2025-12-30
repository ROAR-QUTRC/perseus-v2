#include "arm_driver/arm_driver.hpp"

#include <string>

// IF YOU'RE GOING TO IMPLEMENT SPEED CONTROL, ALSO IMPLEMENT A TIMEOUT TO STOP THE SERVO!!!

using namespace hi_can;
using namespace hi_can::addressing::post_landing;
using namespace hi_can::addressing::post_landing::servo::rmd;
using namespace hi_can::parameters::post_landing::servo::rmd;

ArmDriver::ArmDriver(const rclcpp::NodeOptions& options)
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

    {
        using namespace hi_can::addressing;

        // Handle multi-motor send responses
        _packet_manager->set_callback(
            filter_t{
                .address = servo_address_t{message_type::CANID},
            },
            PacketManager::callback_config_t{
                .data_callback = [this](const Packet& packet)
                {
                    using namespace hi_can::parameters::post_landing::servo::rmd;

                    const auto address = packet.get_address().address;

                    if (address == static_cast<uint16_t>(message_type::CANID))
                    {
                        receive_message::can_id_t can_id_msg;
                        can_id_msg.deserialize_data(packet.get_data());
                        const motor_id_t motor_id = motor_id_t(can_id_msg.can_id);
                        if (std::find(this->_available_servos.begin(), this->_available_servos.end(), motor_id) == this->_available_servos.end())
                        {
                            this->_available_servos.emplace_back(motor_id);
                        }
                    }
                },
            });
    }
    _check_available_servos_timer = this->create_wall_timer(
        std::chrono::seconds(5),
        [this]()
        {
            // Request the ids of all connected servos
            _can_interface->transmit(Packet{
                servo_address_t{message_type::CANID},
                hi_can::parameters::post_landing::servo::rmd::send_message::can_id_t(true).serialize_data()});
            std::this_thread::sleep_for(std::chrono::milliseconds(PACKET_DELAY_MS));
        });

    // Initialise all motors on startup with desired functions
    // Stop all motors - motors will lock up and won't flop around
    _can_interface->transmit(Packet{
        servo_address_t{message_type::MULTI_MOTOR_SEND},
        send_message::action_command_t(send_message::action_command_t::command_id_t::STOP).serialize_data()});
    std::this_thread::sleep_for(std::chrono::milliseconds(PACKET_DELAY_MS));

    // ROS
    _packet_timer = this->create_wall_timer(PACKET_HANDLE, std::bind(&ArmDriver::_can_handle, this));
    _command_subscriber = this->create_subscription<std_msgs::msg::Float64MultiArray>("/arm/rmd_control", 10, std::bind(&ArmDriver::_position_control, this, std::placeholders::_1));
    _status_service = this->create_service<perseus_msgs::srv::RmdServoStatus>("/arm/rmd_status", std::bind(&ArmDriver::_get_rmd_status, this, std::placeholders::_1, std::placeholders::_2));
    _can_id_service = this->create_service<perseus_msgs::srv::RmdCanId>("/arm/rmd_can_ids", std::bind(&ArmDriver::_get_rmd_can_ids, this, std::placeholders::_1, std::placeholders::_2));
    _restart_motor_service = this->create_service<perseus_msgs::srv::RmdData>("/arm/rmd_restart_motor", std::bind(&ArmDriver::_restart_motor, this, std::placeholders::_1, std::placeholders::_2));
    _set_motor_id_service = this->create_service<perseus_msgs::srv::RmdData>("/arm/rmd_set_motor_id", std::bind(&ArmDriver::_set_motor_id, this, std::placeholders::_1, std::placeholders::_2));
    _set_zero_position_service = this->create_service<perseus_msgs::srv::RmdData>("/arm/rmd_set_zero_position", std::bind(&ArmDriver::_set_zero_position, this, std::placeholders::_1, std::placeholders::_2));
    _enable_debug_service = this->create_service<std_srvs::srv::SetBool>(
        "/arm/rmd_enable_status_messages",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response)
        {
            this->_enable_status_messages(request->data);
            this->_status_messages_enabled = request->data;
            response->success = true;
        });

    RCLCPP_INFO(this->get_logger(), "RMD servo driver node initialised");
}

void ArmDriver::_enable_status_messages(bool enable)
{
    // Set motors to transmit error messages when there is an error
    _can_interface->transmit(Packet{
        servo_address_t{message_type::MULTI_MOTOR_SEND},
        send_message::function_control_t(send_message::function_control_t::function_index_t::ERROR_TRANSMISSION_ENABLE, 1).serialize_data()});

    // Set motors to transmit all three status messages every second
    std::this_thread::sleep_for(std::chrono::milliseconds(PACKET_DELAY_MS));
    _can_interface->transmit(Packet{
        servo_address_t{message_type::MULTI_MOTOR_SEND},
        send_message::active_reply_t(send_message::active_reply_t::reply_t::STATUS_1, enable, STATUS_MESSAGE_MS).serialize_data()});
    std::this_thread::sleep_for(std::chrono::milliseconds(PACKET_DELAY_MS));
    _can_interface->transmit(Packet{
        servo_address_t{message_type::MULTI_MOTOR_SEND},
        send_message::active_reply_t(send_message::active_reply_t::reply_t::STATUS_2, enable, STATUS_MESSAGE_MS).serialize_data()});
    std::this_thread::sleep_for(std::chrono::milliseconds(PACKET_DELAY_MS));
    _can_interface->transmit(Packet{
        servo_address_t{message_type::MULTI_MOTOR_SEND},
        send_message::active_reply_t(send_message::active_reply_t::reply_t::STATUS_3, enable, STATUS_MESSAGE_MS).serialize_data()});
    std::this_thread::sleep_for(std::chrono::milliseconds(PACKET_DELAY_MS));  // small delay otherwise messages are ignored by servos
}

void ArmDriver::_position_control(std_msgs::msg::Float64MultiArray servo_control)
{
    for (size_t i = 0; i < servo_control.data.size(); i++)
    {
        RCLCPP_INFO(this->get_logger(), "Setting motor ID %zu to position %.2f", i + 1, servo_control.data[i]);
        motor_id_t motor_id = static_cast<motor_id_t>(i + 1);

        _can_interface->transmit(Packet(
            servo_address_t(message_type::SEND, motor_id),
            send_message::position_t(
                send_message::position_t::position_command_t::ABSOLUTE,
                RMD_SPEED_LIMIT,
                servo_control.data[i])
                .serialize_data()));
    }
}

void ArmDriver::_can_handle()
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

// HANDLE SERVICE REQUESTS

void ArmDriver::_get_rmd_status(const std::shared_ptr<perseus_msgs::srv::RmdServoStatus::Request> request, std::shared_ptr<perseus_msgs::srv::RmdServoStatus::Response> response)
{
    const motor_id_t motor_id = static_cast<motor_id_t>(request->motor_id);

    auto it = this->PARAMETER_GROUP_MAP.find(motor_id);
    if (it != this->PARAMETER_GROUP_MAP.end())
    {
        const auto& parameter_group = it->second;
        response->motor_id = request->motor_id;
        response->temperature = parameter_group->get_temp();
        response->brake_control = parameter_group->get_brake_status() == brake_control_t::BRAKE_RELEASE;
        response->voltage = parameter_group->get_voltage();
        response->torque_current = parameter_group->get_torque_current();
        response->speed = parameter_group->get_speed();
        response->angle = parameter_group->get_angle();
        response->phase_a_current = parameter_group->get_phase_a_current();
        response->phase_b_current = parameter_group->get_phase_b_current();
        response->phase_c_current = parameter_group->get_phase_c_current();
        response->error = static_cast<uint16_t>(parameter_group->get_error_status());
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Requested status for invalid motor ID %d", static_cast<uint8_t>(motor_id));
        return;
    }
}

void ArmDriver::_get_rmd_can_ids(const std::shared_ptr<perseus_msgs::srv::RmdCanId::Request> request, std::shared_ptr<perseus_msgs::srv::RmdCanId::Response> response)
{
    (void)request;  // request is empty

    response->servo_ids.clear();
    for (const auto& motor_id : this->_available_servos)
    {
        response->servo_ids.emplace_back(static_cast<uint8_t>(motor_id));
    }
    RCLCPP_INFO(this->get_logger(), "Returning %zu available RMD servos", response->servo_ids.size());
}

void ArmDriver::_restart_motor(const std::shared_ptr<perseus_msgs::srv::RmdData::Request> request, std::shared_ptr<perseus_msgs::srv::RmdData::Response> response)
{
    _can_interface->transmit(Packet{
        servo_address_t(message_type::SEND, static_cast<motor_id_t>(request->motor_id)),
        send_message::action_command_t(send_message::action_command_t::command_id_t::SHUTDOWN).serialize_data()});
    response->success = true;
}

void ArmDriver::_set_motor_id(const std::shared_ptr<perseus_msgs::srv::RmdData::Request> request, std::shared_ptr<perseus_msgs::srv::RmdData::Response> response)
{
    using function = hi_can::parameters::post_landing::servo::rmd::send_message::function_control_t::function_index_t;

    const motor_id_t new_id = static_cast<motor_id_t>(request->motor_id);

    _can_interface->transmit(Packet{
        servo_address_t(message_type::SEND, new_id),
        send_message::function_control_t(function::SET_CANID, static_cast<uint32_t>(request->data)).serialize_data()});
    std::this_thread::sleep_for(std::chrono::milliseconds(PACKET_DELAY_MS));
    _can_interface->transmit(Packet{
        servo_address_t(message_type::MULTI_MOTOR_SEND),
        send_message::action_command_t(send_message::action_command_t::command_id_t::RESET).serialize_data()});
    std::this_thread::sleep_for(std::chrono::milliseconds(PACKET_DELAY_MS));

    this->_available_servos.erase(std::remove(this->_available_servos.begin(), this->_available_servos.end(), new_id), this->_available_servos.end());

    response->success = true;
}

void ArmDriver::_set_zero_position(const std::shared_ptr<perseus_msgs::srv::RmdData::Request> request, std::shared_ptr<perseus_msgs::srv::RmdData::Response> response)
{
    const motor_id_t motor_id = static_cast<motor_id_t>(request->motor_id);

    _can_interface->transmit(Packet{
        servo_address_t(message_type::SEND, motor_id),
        send_message::zero_offset_t(request->trigger, static_cast<uint32_t>(request->data)).serialize_data()});
    std::this_thread::sleep_for(std::chrono::milliseconds(PACKET_DELAY_MS));
    _can_interface->transmit(Packet{
        servo_address_t(message_type::SEND, static_cast<motor_id_t>(request->motor_id)),
        send_message::action_command_t(send_message::action_command_t::command_id_t::RESET).serialize_data()});
    std::this_thread::sleep_for(std::chrono::milliseconds(PACKET_DELAY_MS));
    response->success = true;
}

void ArmDriver::cleanup()
{
    _packet_manager.reset();
    _can_interface.reset();
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<ArmDriver>();
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