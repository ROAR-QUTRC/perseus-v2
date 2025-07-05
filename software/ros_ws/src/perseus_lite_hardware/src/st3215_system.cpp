#include "st3215_system.hpp"

#include <algorithm>
#include <cmath>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{
    template <typename T>
    bool hasParameter(const std::unordered_map<std::string, T>& params, const std::string& key)
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
        if (_comm_thread_running_)
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
        const hardware_interface::HardwareInfo& info)
    {
        RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), "Initializing ST3215 hardware interface with %zu joints", info.joints.size());
        if (const auto result = SystemInterface::on_init(info);
            result != hardware_interface::CallbackReturn::SUCCESS)
        {
            return result;
        }

        auto logger = rclcpp::get_logger(LOGGER_NAME);

        // Check required parameters
        if (!hasParameter(info.hardware_parameters, "serial_port"))
        {
            RCLCPP_ERROR(logger, "Missing required parameter 'serial_port'");
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (!hasParameter(info.hardware_parameters, "baud_rate"))
        {
            RCLCPP_ERROR(logger, "Missing required parameter 'baud_rate'");
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_DEBUG(logger, "Serial configuration - Port: %s, Baud Rate: %s",
                     info.hardware_parameters.at("serial_port").c_str(),
                     info.hardware_parameters.at("baud_rate").c_str());

        // Pre-allocate vectors with known size
        const auto joint_count = info.joints.size();
        _command_speeds_.resize(joint_count, 0.0);
        _current_positions_.resize(joint_count, 0.0);
        _current_velocities_.resize(joint_count, 0.0);
        _temperatures_.resize(joint_count, ROOM_TEMPERATURE_CELSIUS);  // Initialize to room temperature
        _servo_ids_.reserve(joint_count);

        _servo_states_.resize(joint_count);  // Initialize servo states vector
        _last_update_times_.resize(joint_count, rclcpp::Time(0, 0, RCL_ROS_TIME));

        // Extract and validate servo IDs
        for (const auto& joint : info.joints)
        {
            if (!hasParameter(joint.parameters, "id"))
            {
                RCLCPP_ERROR(logger, "Joint '%s' is missing required parameter 'id'",
                             joint.name.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }

            try
            {
                const auto id = static_cast<uint8_t>(
                    std::stoi(joint.parameters.at("id")));
                _servo_ids_.push_back(id);
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
        for (const auto& joint : info.joints)
        {
            if (!verifyCommandInterfaces(joint, logger))
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
                info.joints[i].name, hardware_interface::HW_IF_POSITION, &_current_positions_[i]));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info.joints[i].name, hardware_interface::HW_IF_VELOCITY, &_current_velocities_[i]));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info.joints[i].name, "temperature", &_temperatures_[i]));
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
                info.joints[i].name, hardware_interface::HW_IF_VELOCITY, &_command_speeds_[i]));
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

            _serial_port_.open(serial_port);

            // Configure serial port settings
            _serial_port_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
            _serial_port_.set_option(boost::asio::serial_port_base::character_size(8));
            _serial_port_.set_option(boost::asio::serial_port_base::stop_bits(
                boost::asio::serial_port_base::stop_bits::one));
            _serial_port_.set_option(boost::asio::serial_port_base::parity(
                boost::asio::serial_port_base::parity::none));
            _serial_port_.set_option(boost::asio::serial_port_base::flow_control(
                boost::asio::serial_port_base::flow_control::none));

            // Set the servos as wheel mode and enable torque
            // Using enum classes instead of #define constants
            const uint8_t modeRegister = static_cast<uint8_t>(ServoEpromRegister::MODE);
            const uint8_t torqueRegister = static_cast<uint8_t>(ServoSramRegister::TORQUE_ENABLE);

            // Set wheel mode and enable torque for each servo
            for (uint8_t servo_id : _servo_ids_)
            {
                RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME),
                             "Setting wheel mode for servo %d", servo_id);

                // Set wheel mode command
                if (!sendServoCommand(servo_id, ServoCommand::WRITE, std::array<uint8_t, 2>{modeRegister, WHEEL_MODE_VALUE}))
                {
                    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                                 "Failed to set wheel mode for servo %d", servo_id);
                    return hardware_interface::CallbackReturn::ERROR;
                }

                // Small delay between commands
                std::this_thread::sleep_for(COMMAND_DELAY);

                RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME),
                             "Enabling torque for servo %d", servo_id);

                // Enable torque command
                if (!sendServoCommand(servo_id, ServoCommand::WRITE, std::array<uint8_t, 2>{torqueRegister, TORQUE_ENABLE_VALUE}))
                {
                    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                                 "Failed to enable torque for servo %d", servo_id);
                    return hardware_interface::CallbackReturn::ERROR;
                }

                // Small delay between servos
                std::this_thread::sleep_for(COMMAND_DELAY);
            }

            RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Successfully configured all servos with wheel mode and torque enabled");

            // Start communication thread
            _comm_thread_running_ = true;
            _comm_thread_ = std::thread(&ST3215SystemHardware::communicationThread, this);

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

    void ST3215SystemHardware::communicationThread() noexcept
    {
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Starting communication thread");

        // Read buffer for responses
        std::array<uint8_t, BUFFER_SIZE> buffer;

        while (_comm_thread_running_)
        {
            // Request status from each servo in sequence
            for (size_t i = 0; i < _servo_ids_.size() && _comm_thread_running_; ++i)
            {
                const auto servo_id = _servo_ids_[i];

                // Create status request packet
                // According to protocol: READ(0x02) command starting at position register (0x38)
                // Reading 8 bytes to get position(2), speed(2), load(2), temp(1), and moving status(1)
                const std::array<uint8_t, 2> read_data{
                    PRESENT_POSITION_REG,    // Start reading from position register (0x38)
                    STATUS_PACKET_DATA_SIZE  // Read 8 bytes total
                };

                {
                    std::lock_guard<std::mutex> lock(_serial_mutex_);
                    if (!sendServoCommand(servo_id, ServoCommand::READ, std::span{read_data}))
                    {
                        RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),
                                    "Failed to request status from servo %d", servo_id);
                        continue;
                    }
                }

                // Give the servo time to process and respond
                std::this_thread::sleep_for(std::chrono::milliseconds(5));

                try
                {
                    // Read response with timeout
                    boost::asio::steady_timer timeout(_io_context_, RESPONSE_TIMEOUT);

                    RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), "Attempting to read response from servo %d", servo_id);
                    boost::system::error_code error;
                    size_t bytes_read = _serial_port_.read_some(
                        boost::asio::buffer(buffer), error);

                    if (error)
                    {
                        RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),
                                    "Error reading from servo %d: %s",
                                    servo_id, error.message().c_str());
                        continue;
                    }

                    if (bytes_read > 0)
                    {
                        processResponse(std::span{buffer.data(), bytes_read});
                    }
                }
                catch (const boost::system::system_error& e)
                {
                    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                                 "Serial communication error: %s", e.what());
                }
            }

            // Wait before starting next update cycle
            std::this_thread::sleep_for(COMMUNICATION_CYCLE_DELAY);
        }

        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Communication thread stopped");
    }

    hardware_interface::CallbackReturn ST3215SystemHardware::on_cleanup(
        const rclcpp_lifecycle::State&)
    {
        // Stop communication thread
        _comm_thread_running_ = false;
        if (_comm_thread_.joinable())
        {
            _comm_thread_.join();
        }

        if (_serial_port_.is_open())
        {
            _serial_port_.close();
        }

        if (_io_thread_.joinable())
        {
            _io_context_.stop();
            _io_thread_.join();
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ST3215SystemHardware::on_activate(
        const rclcpp_lifecycle::State&)
    {
        // Reset command speeds to prevent motion on activation
        std::fill(_command_speeds_.begin(), _command_speeds_.end(), 0.0);
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ST3215SystemHardware::on_deactivate(
        const rclcpp_lifecycle::State&)
    {
        // Stop all servos
        std::fill(_command_speeds_.begin(), _command_speeds_.end(), 0.0);
        auto result = write(rclcpp::Time(), rclcpp::Duration::from_seconds(0));
        return (result == hardware_interface::return_type::OK)
                   ? hardware_interface::CallbackReturn::SUCCESS
                   : hardware_interface::CallbackReturn::ERROR;
    }

    hardware_interface::return_type ST3215SystemHardware::read(
        const rclcpp::Time&, const rclcpp::Duration&)
    {
        RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), "Reading servo states");
        std::lock_guard<std::mutex> lock(_state_mutex_);

        try
        {
            for (size_t i = 0; i < _servo_ids_.size(); ++i)
            {
                const auto& state = _servo_states_[i];

                // Check for timeout and implement recovery
                const auto now = get_clock()->now();
                const double timeout_seconds = (now - state.last_update).seconds();
                if (timeout_seconds > SERVO_TIMEOUT.count())
                {
                    RCLCPP_WARN_THROTTLE(rclcpp::get_logger(LOGGER_NAME),
                                         *get_clock(), 1000,  // Warn every 1 second
                                         "No response from servo %d for more than %ld seconds - implementing recovery",
                                         _servo_ids_[i], SERVO_TIMEOUT.count());
                    
                    // Timeout recovery actions:
                    // 1. Set velocity to zero for safety
                    _current_velocities_[i] = 0.0;
                    
                    // 2. Mark position as stale by not updating it (keep last known position)
                    // This prevents using potentially incorrect position data
                    
                    // 3. Set temperature to a safe default to indicate stale data
                    _temperatures_[i] = ROOM_TEMPERATURE_CELSIUS;
                    
                    // 4. Log detailed timeout information for debugging
                    RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME),
                                "Servo %d timeout recovery: velocity set to 0, position held at %f, temperature reset to %zu",
                                _servo_ids_[i], state.position, ROOM_TEMPERATURE_CELSIUS);
                    
                    // Continue to next servo instead of using stale data
                    continue;
                }

                _current_positions_[i] = state.position;
                _current_velocities_[i] = state.velocity;
                _temperatures_[i] = state.temperature;
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
        std::lock_guard<std::mutex> lock(_serial_mutex_);

        try
        {
            for (size_t i = 0; i < _servo_ids_.size(); ++i)
            {
                // Log input command speed
                RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME),
                             "Servo %d - Input command speed (rad/s): %f",
                             _servo_ids_[i], _command_speeds_[i]);

                // Convert velocity command to servo units
                // ST3215 expects -1000 to 1000 for velocity
                const double normalized_velocity = _command_speeds_[i] * RAD_S_TO_RPM;  // to RPM

                RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME),
                             "Servo %d - Converted to RPM: %f",
                             _servo_ids_[i], normalized_velocity);

                // Debug print the MAX_RPM value being used
                RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME),
                             "Servo %d - Using MAX_RPM value: %f",
                             _servo_ids_[i], MAX_RPM);

                // Normal conversion (comment out when testing fixed speed)
                // Safe conversion with overflow protection
                double scaled_velocity = normalized_velocity * (MAX_VELOCITY_RPM / MAX_RPM);
                double clamped_velocity = std::clamp(scaled_velocity,
                                                     static_cast<double>(MIN_VELOCITY_RPM),
                                                     static_cast<double>(MAX_VELOCITY_RPM));

                // Ensure value is within int16_t range before conversion
                if (clamped_velocity > std::numeric_limits<int16_t>::max())
                {
                    clamped_velocity = std::numeric_limits<int16_t>::max();
                }
                else if (clamped_velocity < std::numeric_limits<int16_t>::min())
                {
                    clamped_velocity = std::numeric_limits<int16_t>::min();
                }

                int16_t servo_speed = static_cast<int16_t>(clamped_velocity);

                RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME),
                             "Servo %d - Calculated servo speed (before direction): %d",
                             _servo_ids_[i], servo_speed);

                // Convert to protocol format (handle negative values per SMS/STS protocol)
                if (servo_speed < 0)
                {
                    servo_speed = -servo_speed;
                    servo_speed |= SIGN_BIT_MASK;  // Set direction bit
                    RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME),
                                 "Servo %d - Negative speed detected, after direction bit: %d",
                                 _servo_ids_[i], servo_speed);
                }

                // Build write command for velocity - format matches SMS_STS::write_speed
                // Using the enum class for the goal speed register
                const uint8_t goalSpeedRegister = static_cast<uint8_t>(ServoSramRegister::GOAL_SPEED_L);
                const std::array<uint8_t, 3> vel_data{
                    goalSpeedRegister,
                    static_cast<uint8_t>(servo_speed & 0xFF),
                    static_cast<uint8_t>((servo_speed >> 8) & 0xFF)};

                // Debug print the final bytes being sent
                RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME),
                             "Servo %d - Final velocity bytes: 0x%02X 0x%02X",
                             _servo_ids_[i], vel_data[1], vel_data[2]);

                if (!sendServoCommand(_servo_ids_[i], ServoCommand::WRITE, std::span{vel_data}))
                {
                    RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),
                                "Failed to send velocity command to servo %d", _servo_ids_[i]);
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

    bool ST3215SystemHardware::sendServoCommand(
        const uint8_t id, const ServoCommand cmd,
        const std::span<const uint8_t> data) noexcept
    {
        std::vector<uint8_t> packet;
        packet.reserve(data.size() + 6);  // Header(2) + ID(1) + Length(1) + CMD(1) + Data(n) + Checksum(1)

        // Build packet
        const std::array<uint8_t, PACKET_HEADER_SIZE> header{PACKET_HEADER_BYTE, PACKET_HEADER_BYTE};
        packet.insert(packet.end(), header.begin(), header.end());
        packet.push_back(id);
        packet.push_back(static_cast<uint8_t>(data.size() + 2));  // Length = data size + cmd(1) + checksum(1)
        packet.push_back(static_cast<uint8_t>(cmd));              // Convert enum class to uint8_t
        packet.insert(packet.end(), data.begin(), data.end());

        // Calculate checksum - XOR of all bytes from ID to the end of data
        const uint8_t checksum = ~std::accumulate(
            packet.begin() + PACKET_ID_INDEX, packet.end(), uint8_t{0});
        packet.push_back(checksum);

        // Debug output - convert to hex string for readable output
        std::stringstream debug_ss;
        debug_ss << "Sending servo command - ID: 0x" << std::hex << static_cast<int>(id)
                 << " CMD: 0x" << static_cast<int>(static_cast<uint8_t>(cmd)) << " Packet: ";
        for (const auto& byte : packet)
        {
            debug_ss << "0x" << std::setw(2) << std::setfill('0')
                     << static_cast<int>(byte) << " ";
        }
        RCLCPP_INFO(rclcpp::get_logger("ST3215Hardware"), "%s", debug_ss.str().c_str());

        try
        {
            boost::asio::write(_serial_port_, boost::asio::buffer(packet));
            return true;
        }
        catch (const boost::system::system_error& e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ST3215Hardware"),
                         "Failed to send command: %s", e.what());
            return false;
        }
    }

    void ST3215SystemHardware::processResponse(const std::span<const uint8_t> response) noexcept
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

        // Minimum packet size: header(2) + ID(1) + length(1) = 4 bytes
        if (response.size() < PACKET_MIN_SIZE)
        {
            RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME),
                         "Response too short (%zu bytes), ignoring", response.size());
            return;
        }

        // Loop through response looking for valid packets
        for (size_t i = 0; i < response.size() - 3; ++i)
        {
            // Look for packet header (0xFF 0xFF)
            if (response[i] != PACKET_HEADER_BYTE || response[i + 1] != PACKET_HEADER_BYTE)
            {
                continue;
            }

            const uint8_t id = response[i + PACKET_ID_INDEX];
            const uint8_t length = response[i + PACKET_LENGTH_INDEX];

            // Validate packet length
            if (i + PACKET_MIN_SIZE + length > response.size())
            {
                RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME),
                             "Incomplete packet for ID %d: expected %d bytes, have %zu",
                             id, length, response.size() - (i + PACKET_MIN_SIZE));
                continue;
            }

            // Validate checksum
            uint8_t checksum = 0;
            for (size_t j = i + PACKET_ID_INDEX; j < i + PACKET_MIN_SIZE + length - 1; ++j)
            {
                checksum += response[j];
            }
            checksum = ~checksum;

            if (checksum != response[i + PACKET_MIN_SIZE + length - 1])
            {
                RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME),
                             "Checksum mismatch for ID %d: expected 0x%02X, got 0x%02X",
                             id, checksum, response[i + PACKET_MIN_SIZE + length - 1]);
                continue;
            }

            // Find matching servo ID
            const auto it = std::find(_servo_ids_.begin(), _servo_ids_.end(), id);
            if (it == _servo_ids_.end())
            {
                RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME),
                             "Received packet for unknown servo ID %d", id);
                continue;
            }

            const auto index = std::distance(_servo_ids_.begin(), it);
            RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME),
                         "Processing packet for servo ID %d (index %ld)", id, index);

            // Extract packet data (skip header, ID, length)
            const std::span packet{response.data() + i + PACKET_MIN_SIZE, static_cast<size_t>(length)};

            // Validate index bounds before accessing arrays
            if (static_cast<size_t>(index) >= _servo_states_.size())
            {
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                             "Invalid servo index %ld for ID %d (max: %zu)",
                             index, id, _servo_states_.size());
                continue;
            }

            // Lock state mutex while updating
            std::lock_guard<std::mutex> state_lock(_state_mutex_);
            auto& state = _servo_states_[index];

            // Update timestamp
            state.last_update = rclcpp::Clock(RCL_ROS_TIME).now();

            // Process based on response type
            if (packet.size() > 0)
            {
                const uint8_t error_byte = packet[ERROR_BYTE_INDEX];

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
                    // Don't return - we can still process the data
                }

                // Check if this is a status response (should have at least 8 bytes of data)
                if (packet.size() >= STATUS_PACKET_DATA_SIZE)
                {
                    // Extract position (2 bytes, little endian) with safe conversion
                    // Ensure we have valid indices before accessing packet data
                    if (POSITION_LOW_BYTE_INDEX < packet.size() && POSITION_HIGH_BYTE_INDEX < packet.size())
                    {
                        uint16_t raw_pos_unsigned = static_cast<uint16_t>(
                            packet[POSITION_LOW_BYTE_INDEX] | (static_cast<uint16_t>(packet[POSITION_HIGH_BYTE_INDEX]) << 8));

                        // Safe conversion to signed integer
                        int16_t raw_pos = static_cast<int16_t>(raw_pos_unsigned);

                        // Handle position according to protocol (12-bit resolution)
                        if (raw_pos & SIGN_BIT_MASK)
                        {  // Check sign bit
                            raw_pos = -(raw_pos & ~SIGN_BIT_MASK);
                        }
                        // Convert to radians (4096 counts per revolution)
                        state.position = raw_pos * (RADIANS_PER_REVOLUTION / ENCODER_TICKS_PER_REVOLUTION);
                    }

                    // Extract velocity (2 bytes, little endian) with safe conversion
                    if (VELOCITY_LOW_BYTE_INDEX < packet.size() && VELOCITY_HIGH_BYTE_INDEX < packet.size())
                    {
                        uint16_t raw_vel_unsigned = static_cast<uint16_t>(
                            packet[VELOCITY_LOW_BYTE_INDEX] | (static_cast<uint16_t>(packet[VELOCITY_HIGH_BYTE_INDEX]) << 8));

                        // Safe conversion to signed integer
                        int16_t raw_vel = static_cast<int16_t>(raw_vel_unsigned);

                        // Handle velocity according to protocol (-1000 to 1000)
                        if (raw_vel & SIGN_BIT_MASK)
                        {  // Check sign bit
                            raw_vel = -(raw_vel & ~SIGN_BIT_MASK);
                        }
                        // Convert to rad/s (protocol units are roughly RPM/1000)
                        const double rpm = raw_vel * (MAX_RPM / MAX_VELOCITY_RPM);
                        state.velocity = rpm * RPM_TO_RAD_S;
                    }

                    // Extract temperature (1 byte)
                    state.temperature = static_cast<double>(packet[TEMPERATURE_BYTE_INDEX]);

                    RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME),
                                 "Servo %d state updated - Pos: %.2f rad, Vel: %.2f rad/s, Temp: %.1f°C",
                                 id, state.position, state.velocity, state.temperature);
                }
            }

            // Skip to end of this packet
            i += 3 + length;
        }
    }

    bool ST3215SystemHardware::updateServoStates(uint8_t id, size_t index) noexcept
    {
        try
        {
            // Validate index bounds before accessing arrays
            if (index >= _last_update_times_.size() ||
                index >= _current_positions_.size() ||
                index >= _current_velocities_.size() ||
                index >= _temperatures_.size())
            {
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                             "Invalid servo index %zu for ID %d (max arrays: %zu, %zu, %zu, %zu)",
                             index, id, _last_update_times_.size(), _current_positions_.size(),
                             _current_velocities_.size(), _temperatures_.size());
                return false;
            }

            // Update the timestamp first
            _last_update_times_[index] = rclcpp::Clock(RCL_ROS_TIME).now();

            _current_positions_[index] = 0.0;                  // Replace with actual position reading
            _current_velocities_[index] = 0.0;                 // Replace with actual velocity reading
            _temperatures_[index] = ROOM_TEMPERATURE_CELSIUS;  // Replace with actual temperature reading

            return true;
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                         "Error updating servo %d states: %s", id, e.what());
            return false;
        }
    }

    bool ST3215SystemHardware::verifyCommandInterfaces(
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

    void ST3215SystemHardware::updateServoStates() noexcept
    {
        // This is an empty implementation of the declared method in the header
        // It might be used in future implementations or for more complex state updates
    }

}  // namespace perseus_lite_hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    perseus_lite_hardware::ST3215SystemHardware,
    hardware_interface::SystemInterface)