#include "st3215_system.hpp"

#include <algorithm>
#include <cmath>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include "st3215_protocol.hpp"

namespace
{
    template <typename T>
    bool has_parameter(const std::unordered_map<std::string, T>& params, const std::string& key)
    {
        return params.find(key) != params.end();
    }
}

namespace perseus_lite_hardware
{
    namespace
    {
        static const char* const LOGGER_NAME = "ST3215SystemHardware";
    }  // namespace

    ST3215SystemHardware::~ST3215SystemHardware()
    {
        if (_comm_thread_running)
        {
            auto deactivate_result = on_deactivate(rclcpp_lifecycle::State());
            if (deactivate_result != hardware_interface::CallbackReturn::SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Failed to deactivate");
            }
        }

        auto cleanup_result = on_cleanup(rclcpp_lifecycle::State());
        if (cleanup_result != hardware_interface::CallbackReturn::SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Failed to cleanup");
        }
    }

    hardware_interface::CallbackReturn ST3215SystemHardware::on_init(
        const hardware_interface::HardwareComponentInterfaceParams& params)
    {
        RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), "Initializing ST3215 hardware interface with %zu joints", params.hardware_info.joints.size());
        if (const auto result = SystemInterface::on_init(params);
            result != hardware_interface::CallbackReturn::SUCCESS)
        {
            return result;
        }

        auto logger = rclcpp::get_logger(LOGGER_NAME);

        // Check required parameters
        if (!has_parameter(params.hardware_info.hardware_parameters, "serial_port"))
        {
            RCLCPP_ERROR(logger, "Missing required parameter 'serial_port'");
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (!has_parameter(params.hardware_info.hardware_parameters, "baud_rate"))
        {
            RCLCPP_ERROR(logger, "Missing required parameter 'baud_rate'");
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_DEBUG(logger, "Serial configuration - Port: %s, Baud Rate: %s",
                     params.hardware_info.hardware_parameters.at("serial_port").c_str(),
                     params.hardware_info.hardware_parameters.at("baud_rate").c_str());

        // Pre-allocate vectors with known size
        const auto joint_count = params.hardware_info.joints.size();
        _command_speeds.resize(joint_count, 0.0);
        _current_positions.resize(joint_count, 0.0);
        _current_velocities.resize(joint_count, 0.0);
        _temperatures.resize(joint_count, _ROOM_TEMPERATURE_CELSIUS);  // Initialize to room temperature
        _servo_ids.reserve(joint_count);

        _servo_states.resize(joint_count);  // Initialize servo states vector

        // Servos whose drive direction is inverted (left side of the rover).
        // Override with the hardware parameter 'inverted_servo_ids', a
        // comma-separated ID list, when the wiring differs.
        _inverted_servo_ids = {2, 3};
        if (has_parameter(params.hardware_info.hardware_parameters, "inverted_servo_ids"))
        {
            _inverted_servo_ids.clear();
            std::stringstream ids_ss(params.hardware_info.hardware_parameters.at("inverted_servo_ids"));
            std::string token;
            while (std::getline(ids_ss, token, ','))
            {
                try
                {
                    _inverted_servo_ids.push_back(static_cast<uint8_t>(std::stoi(token)));
                }
                catch (const std::exception& e)
                {
                    RCLCPP_ERROR(logger, "Invalid 'inverted_servo_ids' entry '%s': %s",
                                 token.c_str(), e.what());
                    return hardware_interface::CallbackReturn::ERROR;
                }
            }
        }

        // Extract and validate servo IDs
        for (const auto& joint : params.hardware_info.joints)
        {
            if (!has_parameter(joint.parameters, "id"))
            {
                RCLCPP_ERROR(logger, "Joint '%s' is missing required parameter 'id'",
                             joint.name.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }

            try
            {
                const auto id = static_cast<uint8_t>(
                    std::stoi(joint.parameters.at("id")));
                _servo_ids.push_back(id);
            }
            catch (const std::exception& e)
            {
                RCLCPP_FATAL(logger,
                             "Failed to parse ID for joint '%s': %s",
                             joint.name.c_str(), e.what());
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        // Verify command interfaces
        for (const auto& joint : params.hardware_info.joints)
        {
            if (!verify_command_interfaces(joint, logger))
            {
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> ST3215SystemHardware::export_state_interfaces()
    {
        RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), "Exporting state interfaces");

        std::vector<hardware_interface::StateInterface> state_interfaces;
        const auto& info = get_hardware_info();

        for (size_t i = 0; i < info.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info.joints[i].name, hardware_interface::HW_IF_POSITION, &_current_positions[i]));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info.joints[i].name, hardware_interface::HW_IF_VELOCITY, &_current_velocities[i]));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info.joints[i].name, hardware_interface::HW_IF_TEMPERATURE, &_temperatures[i]));
        }

        RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), "Exported %zu state interfaces", state_interfaces.size());
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> ST3215SystemHardware::export_command_interfaces()
    {
        RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), "Exporting command interfaces");

        std::vector<hardware_interface::CommandInterface> command_interfaces;
        const auto& info = get_hardware_info();

        for (size_t i = 0; i < info.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info.joints[i].name, hardware_interface::HW_IF_VELOCITY, &_command_speeds[i]));
        }

        RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), "Exported %zu command interfaces", command_interfaces.size());
        return command_interfaces;
    }

    hardware_interface::CallbackReturn ST3215SystemHardware::on_configure(
        const rclcpp_lifecycle::State&)
    {
        try
        {
            const auto& serial_port = info_.hardware_parameters.at("serial_port");
            const int baud_rate = std::stoi(info_.hardware_parameters.at("baud_rate"));

            _serial_port.open(serial_port);

            // Configure serial port settings
            _serial_port.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
            _serial_port.set_option(boost::asio::serial_port_base::character_size(8));
            _serial_port.set_option(boost::asio::serial_port_base::stop_bits(
                boost::asio::serial_port_base::stop_bits::one));
            _serial_port.set_option(boost::asio::serial_port_base::parity(
                boost::asio::serial_port_base::parity::none));
            _serial_port.set_option(boost::asio::serial_port_base::flow_control(
                boost::asio::serial_port_base::flow_control::none));

            // Set the servos as wheel mode and enable torque
            // Using enum classes instead of #define constants
            const uint8_t mode_register = static_cast<uint8_t>(ServoEpromRegister::MODE);
            const uint8_t torque_register = static_cast<uint8_t>(ServoSramRegister::TORQUE_ENABLE);

            // Set wheel mode and enable torque for each servo
            for (uint8_t servo_id : _servo_ids)
            {
                RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME),
                             "Setting wheel mode for servo %d", servo_id);

                // Set wheel mode command
                if (!send_servo_command(servo_id, ServoCommand::WRITE, std::array<uint8_t, 2>{mode_register, _WHEEL_MODE_VALUE}))
                {
                    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                                 "Failed to set wheel mode for servo %d", servo_id);
                    return hardware_interface::CallbackReturn::ERROR;
                }

                // Small delay between commands
                std::this_thread::sleep_for(_COMMAND_DELAY);

                RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME),
                             "Enabling torque for servo %d", servo_id);

                // Enable torque command
                if (!send_servo_command(servo_id, ServoCommand::WRITE, std::array<uint8_t, 2>{torque_register, _TORQUE_ENABLE_VALUE}))
                {
                    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                                 "Failed to enable torque for servo %d", servo_id);
                    return hardware_interface::CallbackReturn::ERROR;
                }

                // Small delay between servos
                std::this_thread::sleep_for(_COMMAND_DELAY);
            }

            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Successfully configured all servos with wheel mode and torque enabled");

            // Start communication thread
            _comm_thread_running = true;
            _comm_thread = std::thread(&ST3215SystemHardware::communication_thread, this);

            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Successfully configured!");
            return hardware_interface::CallbackReturn::SUCCESS;
        }
        catch (const boost::system::system_error& e)
        {
            RCLCPP_FATAL(rclcpp::get_logger(LOGGER_NAME),
                         "Serial port error: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    void ST3215SystemHardware::communication_thread() noexcept
    {
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Starting communication thread");

        // Read buffer for responses
        std::array<uint8_t, _BUFFER_SIZE> buffer;

        while (_comm_thread_running)
        {
            // Request status from each servo in sequence
            for (size_t i = 0; i < _servo_ids.size() && _comm_thread_running; ++i)
            {
                const auto servo_id = _servo_ids[i];

                // Create status request packet
                // According to protocol: READ(0x02) command starting at position register (0x38)
                // Reading 8 bytes to get position(2), speed(2), load(2), temp(1), and moving status(1)
                const std::array<uint8_t, 2> read_data{
                    _PRESENT_POSITION_REG,                              // Start reading from position register (0x38)
                    static_cast<uint8_t>(protocol::STATUS_DATA_SIZE)};  // Read 8 bytes total

                try
                {
                    // Hold the serial mutex across the whole request/response
                    // transaction so a velocity write() from the controller
                    // thread cannot interleave between request and reply.
                    std::lock_guard<std::mutex> lock(_serial_mutex);
                    if (!send_servo_command(servo_id, ServoCommand::READ, std::span{read_data}))
                    {
                        RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),
                                    "Failed to request status from servo %d", servo_id);
                        continue;
                    }

                    // Give the servo time to process and respond so the reply
                    // is complete when the read returns
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));

                    RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), "Attempting to read response from servo %d", servo_id);
                    boost::system::error_code error;
                    const size_t bytes_read = read_with_timeout(buffer, _RESPONSE_TIMEOUT, error);

                    if (error == boost::asio::error::operation_aborted)
                    {
                        static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
                        RCLCPP_WARN_THROTTLE(rclcpp::get_logger(LOGGER_NAME),
                                             steady_clock, 1000,
                                             "Timed out waiting for response from servo %d", servo_id);
                        continue;
                    }
                    if (error)
                    {
                        RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),
                                    "Error reading from servo %d: %s",
                                    servo_id, error.message().c_str());
                        continue;
                    }

                    if (bytes_read > 0)
                    {
                        process_response(std::span{buffer.data(), bytes_read});
                    }
                }
                catch (const boost::system::system_error& e)
                {
                    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                                 "Serial communication error: %s", e.what());
                }
            }

            // Wait before starting next update cycle
            std::this_thread::sleep_for(_COMMUNICATION_CYCLE_DELAY);
        }

        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Communication thread stopped");
    }

    size_t ST3215SystemHardware::read_with_timeout(std::span<uint8_t> buffer,
                                                   std::chrono::milliseconds timeout,
                                                   boost::system::error_code& error)
    {
        size_t bytes_read = 0;
        error = boost::asio::error::operation_aborted;

        boost::asio::steady_timer deadline(_io_context, timeout);

        _serial_port.async_read_some(
            boost::asio::buffer(buffer.data(), buffer.size()),
            [&](const boost::system::error_code& ec, size_t n)
            {
                error = ec;
                bytes_read = n;
                deadline.cancel();
            });

        deadline.async_wait(
            [&](const boost::system::error_code& ec)
            {
                // Timer fired (not cancelled by a completed read): abort the read
                if (ec != boost::asio::error::operation_aborted)
                {
                    _serial_port.cancel();
                }
            });

        _io_context.restart();
        _io_context.run();

        return error ? 0 : bytes_read;
    }

    hardware_interface::CallbackReturn ST3215SystemHardware::on_cleanup(
        const rclcpp_lifecycle::State&)
    {
        // Stop communication thread
        _comm_thread_running = false;
        if (_comm_thread.joinable())
        {
            _comm_thread.join();
        }

        if (_serial_port.is_open())
        {
            _serial_port.close();
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ST3215SystemHardware::on_activate(
        const rclcpp_lifecycle::State&)
    {
        // Reset command speeds to prevent motion on activation
        std::fill(_command_speeds.begin(), _command_speeds.end(), 0.0);
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ST3215SystemHardware::on_deactivate(
        const rclcpp_lifecycle::State&)
    {
        // Stop all servos
        std::fill(_command_speeds.begin(), _command_speeds.end(), 0.0);
        auto result = write(rclcpp::Time(), rclcpp::Duration::from_seconds(0));
        return (result == hardware_interface::return_type::OK)
                   ? hardware_interface::CallbackReturn::SUCCESS
                   : hardware_interface::CallbackReturn::ERROR;
    }

    hardware_interface::return_type ST3215SystemHardware::read(
        const rclcpp::Time&, const rclcpp::Duration&)
    {
        RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), "Reading servo states");
        std::lock_guard<std::mutex> lock(_state_mutex);

        try
        {
            for (size_t i = 0; i < _servo_ids.size(); ++i)
            {
                const auto& state = _servo_states[i];

                // Check for timeout and implement recovery
                const auto now = get_clock()->now();
                const double timeout_seconds = (now - state.last_update).seconds();
                if (timeout_seconds > _SERVO_TIMEOUT.count())
                {
                    RCLCPP_WARN_THROTTLE(rclcpp::get_logger(LOGGER_NAME),
                                         *get_clock(), 1000,  // Warn every 1 second
                                         "No response from servo %d for more than %ld seconds - implementing recovery",
                                         _servo_ids[i], _SERVO_TIMEOUT.count());

                    // Timeout recovery actions:
                    // 1. Set velocity to zero for safety
                    _current_velocities[i] = 0.0;

                    // 2. Mark position as stale by not updating it (keep last known position)
                    // This prevents using potentially incorrect position data

                    // 3. Set temperature to a safe default to indicate stale data
                    _temperatures[i] = _ROOM_TEMPERATURE_CELSIUS;

                    // 4. Log detailed timeout information for debugging
                    RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME),
                                 "Servo %d timeout recovery: velocity set to 0, position held at %f, temperature reset to %zu",
                                 _servo_ids[i], state.position, _ROOM_TEMPERATURE_CELSIUS);

                    // Continue to next servo instead of using stale data
                    continue;
                }

                _current_positions[i] = state.position;
                _current_velocities[i] = state.velocity;
                _temperatures[i] = state.temperature;
            }

            return hardware_interface::return_type::OK;
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                         "Error reading from servos: %s", e.what());
            return hardware_interface::return_type::ERROR;
        }
    }

    hardware_interface::return_type ST3215SystemHardware::write(
        const rclcpp::Time&, const rclcpp::Duration&)
    {
        RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), "Writing servo commands");
        std::lock_guard<std::mutex> lock(_serial_mutex);

        try
        {
            for (size_t i = 0; i < _servo_ids.size(); ++i)
            {
                RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME),
                             "Servo %d - Input command speed (rad/s): %f",
                             _servo_ids[i], _command_speeds[i]);

                double corrected_speed = protocol::apply_motor_direction(
                    _servo_ids[i], _command_speeds[i], _inverted_servo_ids);

                uint16_t servo_speed = protocol::encode_servo_velocity(corrected_speed);

                const uint8_t goal_speed_register = static_cast<uint8_t>(ServoSramRegister::GOAL_SPEED_L);
                const std::array<uint8_t, 3> vel_data{
                    goal_speed_register,
                    static_cast<uint8_t>(servo_speed & 0xFF),
                    static_cast<uint8_t>((servo_speed >> 8) & 0xFF)};

                RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME),
                             "Servo %d - Final velocity bytes: 0x%02X 0x%02X",
                             _servo_ids[i], vel_data[1], vel_data[2]);

                if (!send_servo_command(_servo_ids[i], ServoCommand::WRITE, std::span{vel_data}))
                {
                    RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),
                                "Failed to send velocity command to servo %d", _servo_ids[i]);
                    return hardware_interface::return_type::ERROR;
                }
            }

            return hardware_interface::return_type::OK;
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                         "Error writing to servos: %s", e.what());
            return hardware_interface::return_type::ERROR;
        }
    }

    bool ST3215SystemHardware::send_servo_command(
        const uint8_t id, const ServoCommand command,
        const std::span<const uint8_t> data) noexcept
    {
        auto packet = protocol::build_packet(
            id, static_cast<protocol::Command>(static_cast<uint8_t>(command)), data);

        // Debug output - convert to hex string for readable output
        std::stringstream debug_ss;
        debug_ss << "Sending servo command - ID: 0x" << std::hex << static_cast<int>(id)
                 << " CMD: 0x" << static_cast<int>(static_cast<uint8_t>(command)) << " Packet: ";
        for (const auto& byte : packet)
        {
            debug_ss << "0x" << std::setw(2) << std::setfill('0')
                     << static_cast<int>(byte) << " ";
        }
        RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), "%s", debug_ss.str().c_str());

        try
        {
            boost::asio::write(_serial_port, boost::asio::buffer(packet));
            return true;
        }
        catch (const boost::system::system_error& e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ST3215Hardware"),
                         "Failed to send command: %s", e.what());
            return false;
        }
    }

    void ST3215SystemHardware::process_response(const std::span<const uint8_t> response) noexcept
    {
        // Log raw response bytes for debugging
        {
            std::stringstream debug_ss;
            debug_ss << "Raw response bytes: ";
            for (const auto& byte : response)
            {
                debug_ss << std::hex << std::setw(2) << std::setfill('0')
                         << static_cast<int>(byte) << " ";
            }
            RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), "%s", debug_ss.str().c_str());
        }

        for (const auto& [id, payload] : protocol::extract_packets(response))
        {
            // Find matching servo ID
            const auto it = std::find(_servo_ids.begin(), _servo_ids.end(), id);
            if (it == _servo_ids.end())
            {
                RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME),
                             "Received packet for unknown servo ID %d", id);
                continue;
            }

            const auto index = static_cast<size_t>(std::distance(_servo_ids.begin(), it));
            RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME),
                         "Processing packet for servo ID %d (index %zu)", id, index);

            if (!payload.empty())
            {
                const uint8_t error_byte = payload[protocol::ERROR_BYTE_INDEX];

                // Check error flags if present
                if (error_byte != 0)
                {
                    if (error_byte & static_cast<uint8_t>(ServoErrorFlag::INPUT_VOLTAGE))
                        RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),
                                    "Servo %d: Input Voltage Error", id);
                    if (error_byte & static_cast<uint8_t>(ServoErrorFlag::ANGLE_LIMIT))
                        RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),
                                    "Servo %d: Angle Limit Error", id);
                    if (error_byte & static_cast<uint8_t>(ServoErrorFlag::OVERHEATING))
                        RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),
                                    "Servo %d: Overheating Error", id);
                    if (error_byte & static_cast<uint8_t>(ServoErrorFlag::RANGE))
                        RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),
                                    "Servo %d: Range Error", id);
                    if (error_byte & static_cast<uint8_t>(ServoErrorFlag::CHECKSUM))
                        RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),
                                    "Servo %d: Checksum Error", id);
                    if (error_byte & static_cast<uint8_t>(ServoErrorFlag::OVERLOAD))
                        RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),
                                    "Servo %d: Overload Error", id);
                    if (error_byte & static_cast<uint8_t>(ServoErrorFlag::INSTRUCTION))
                        RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),
                                    "Servo %d: Instruction Error", id);
                }
            }

            // Lock state mutex while updating
            std::lock_guard<std::mutex> state_lock(_state_mutex);
            auto& state = _servo_states[index];

            // Update timestamp
            state.last_update = rclcpp::Clock(RCL_ROS_TIME).now();

            if (const auto status = protocol::parse_status_payload(payload))
            {
                state.position = status->position_rad;
                state.velocity = status->velocity_rad_s;
                state.temperature = status->temperature_c;

                RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME),
                             "Servo %d state updated - Pos: %.2f rad, Vel: %.2f rad/s, Temp: %.1f°C",
                             id, state.position, state.velocity, state.temperature);
            }
        }
    }

    bool ST3215SystemHardware::verify_command_interfaces(
        const hardware_interface::ComponentInfo& joint_info,
        const rclcpp::Logger& logger) const
    {
        if (joint_info.command_interfaces.size() != 1)
        {
            RCLCPP_ERROR(logger,
                         "Joint '%s' has %zu command interfaces. Expected 1 (velocity)",
                         joint_info.name.c_str(), joint_info.command_interfaces.size());
            return false;
        }

        // Verify velocity interface
        if (joint_info.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_ERROR(logger,
                         "Joint '%s' missing velocity command interface",
                         joint_info.name.c_str());
            return false;
        }

        return true;
    }

}  // namespace perseus_lite_hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    perseus_lite_hardware::ST3215SystemHardware,
    hardware_interface::SystemInterface)