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
        constexpr auto READ_TIMEOUT = std::chrono::milliseconds(10);
        constexpr auto WRITE_TIMEOUT = std::chrono::milliseconds(1);
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
        // _command_positions_.resize(joint_count, 0.0);
        _command_speeds_.resize(joint_count, 0.0);
        _current_positions_.resize(joint_count, 0.0);
        _current_velocities_.resize(joint_count, 0.0);
        // _current_loads_.resize(joint_count, 0.0);
        _temperatures_.resize(joint_count, 25.0);  // Initialize to room temperature
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

            // state_interfaces.emplace_back(hardware_interface::StateInterface(
            //     info.joints[i].name, "load", &_current_loads_[i]));

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

            // set the serves as wheel mode and enable torque
            const uint8_t MODE_REGISTER = 33;    // SMS_STS_MODE register
            const uint8_t TORQUE_REGISTER = 40;  // SMS_STS_TORQUE_ENABLE register
            const uint8_t WHEEL_MODE_VALUE = 1;  // tested in FT tool - then set goal velocity (46)
            const uint8_t TORQUE_ENABLE_VALUE = 1;
            const uint8_t CMD_WRITE = 0x03;

            // Set wheel mode and enable torque for each servo
            for (uint8_t servo_id : _servo_ids_)
            {
                RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME),
                             "Setting wheel mode for servo %d", servo_id);

                // Set wheel mode command
                if (!sendServoCommand(servo_id, CMD_WRITE, std::array<uint8_t, 2>{MODE_REGISTER, WHEEL_MODE_VALUE}))
                {
                    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                                 "Failed to set wheel mode for servo %d", servo_id);
                    return hardware_interface::CallbackReturn::ERROR;
                }

                // Small delay between commands
                std::this_thread::sleep_for(std::chrono::milliseconds(10));

                RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME),
                             "Enabling torque for servo %d", servo_id);

                // Enable torque command
                if (!sendServoCommand(servo_id, CMD_WRITE, std::array<uint8_t, 2>{TORQUE_REGISTER, TORQUE_ENABLE_VALUE}))
                {
                    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                                 "Failed to enable torque for servo %d", servo_id);
                    return hardware_interface::CallbackReturn::ERROR;
                }

                // Small delay between servos
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
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
                    REG_PRESENT_POSITION,  // Start reading from position register (0x38)
                    8                      // Read 8 bytes total
                };

                {
                    std::lock_guard<std::mutex> lock(_serial_mutex_);
                    if (!sendServoCommand(servo_id, CMD_READ, std::span{read_data}))
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
                    boost::asio::steady_timer timeout(_io_context_,
                                                      std::chrono::milliseconds(50));  // 50ms timeout

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
            // Adjust this delay based on your required update frequency
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
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

                // Check for timeout
                const auto now = get_clock()->now();
                if ((now - state.last_update).seconds() > SERVO_TIMEOUT.count())
                {
                    RCLCPP_WARN_THROTTLE(rclcpp::get_logger(LOGGER_NAME),
                                         *get_clock(), 1000,  // Warn every 1 second
                                         "No response from servo %d for more than %ld seconds",
                                         _servo_ids_[i], SERVO_TIMEOUT.count());
                }

                _current_positions_[i] = state.position;
                _current_velocities_[i] = state.velocity;
                // _current_loads_[i] = state.load;
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
                RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),
                            "Servo %d - Input command speed (rad/s): %f",
                            _servo_ids_[i], _command_speeds_[i]);

                // Convert velocity command to servo units
                // ST3215 expects -1000 to 1000 for velocity
                const double normalized_velocity = _command_speeds_[i] * (60.0 / (2.0 * M_PI));  // to RPM

                RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),
                            "Servo %d - Converted to RPM: %f",
                            _servo_ids_[i], normalized_velocity);

                // Debug print the MAX_RPM value being used
                RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),
                            "Servo %d - Using MAX_RPM value: %f",
                            _servo_ids_[i], MAX_RPM);

                // For testing, use a fixed value instead of the conversion
                // int16_t servo_speed = 500;  // Uncomment this line to test with fixed speed

                // Normal conversion (comment out when testing fixed speed)
                int16_t servo_speed = static_cast<int16_t>(
                    std::clamp(normalized_velocity * (1000.0 / MAX_RPM), -1000.0, 1000.0));

                RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),
                            "Servo %d - Calculated servo speed (before direction): %d",
                            _servo_ids_[i], servo_speed);

                // Convert to protocol format (handle negative values per SMS/STS protocol)
                if (servo_speed < 0)
                {
                    servo_speed = -servo_speed;
                    servo_speed |= (1 << 15);  // Set direction bit
                    RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME),
                                 "Servo %d - Negative speed detected, after direction bit: %d",
                                 _servo_ids_[i], servo_speed);
                }

                // Build write command for velocity - format matches SMS_STS::write_speed
                const std::array<uint8_t, 3> vel_data{
                    REG_GOAL_SPEED_L,
                    static_cast<uint8_t>(servo_speed & 0xFF),
                    static_cast<uint8_t>((servo_speed >> 8) & 0xFF)};

                // Debug print the final bytes being sent
                RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),
                            "Servo %d - Final velocity bytes: 0x%02X 0x%02X",
                            _servo_ids_[i], vel_data[1], vel_data[2]);

                if (!sendServoCommand(_servo_ids_[i], CMD_WRITE, std::span{vel_data}))
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

    // temp test of sync not async
    bool ST3215SystemHardware::sendServoCommand(
        const uint8_t id, const uint8_t cmd,
        const std::span<const uint8_t> data) noexcept
    {
        std::vector<uint8_t> packet;
        packet.reserve(data.size() + 6);

        // Build packet
        const std::array<uint8_t, 2> header{0xFF, 0xFF};
        packet.insert(packet.end(), header.begin(), header.end());
        packet.push_back(id);
        packet.push_back(static_cast<uint8_t>(data.size() + 2));
        packet.push_back(cmd);
        packet.insert(packet.end(), data.begin(), data.end());

        // Calculate checksum
        const uint8_t checksum = ~std::accumulate(
            packet.begin() + 2, packet.end(), uint8_t{0});
        packet.push_back(checksum);

        // Debug output - convert to hex string for readable output
        std::stringstream debug_ss;
        debug_ss << "Sending servo command - ID: 0x" << std::hex << static_cast<int>(id)
                 << " CMD: 0x" << static_cast<int>(cmd) << " Packet: ";
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
        if (response.size() < 4)
        {
            RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME),
                         "Response too short (%zu bytes), ignoring", response.size());
            return;
        }

        // Loop through response looking for valid packets
        for (size_t i = 0; i < response.size() - 3; ++i)
        {
            // Look for packet header (0xFF 0xFF)
            if (response[i] != 0xFF || response[i + 1] != 0xFF)
            {
                continue;
            }

            const uint8_t id = response[i + 2];
            const uint8_t length = response[i + 3];

            // Validate packet length
            if (i + 4 + length > response.size())
            {
                RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME),
                             "Incomplete packet for ID %d: expected %d bytes, have %zu",
                             id, length, response.size() - (i + 4));
                continue;
            }

            // Validate checksum
            uint8_t checksum = 0;
            for (size_t j = i + 2; j < i + 4 + length - 1; ++j)
            {
                checksum += response[j];
            }
            checksum = ~checksum;

            if (checksum != response[i + 4 + length - 1])
            {
                RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME),
                             "Checksum mismatch for ID %d: expected 0x%02X, got 0x%02X",
                             id, checksum, response[i + 4 + length - 1]);
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
            const std::span packet{response.data() + i + 4, static_cast<size_t>(length)};

            // Lock state mutex while updating
            std::lock_guard<std::mutex> state_lock(_state_mutex_);
            auto& state = _servo_states_[index];

            // Update timestamp
            state.last_update = rclcpp::Clock(RCL_ROS_TIME).now();

            // Process based on response type
            if (packet.size() > 0)
            {
                const uint8_t error_byte = packet[0];

                // Check error flags if present
                if (error_byte != 0)
                {
                    if (error_byte & 0x01)
                        RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),
                                    "Servo %d: Input Voltage Error", id);
                    if (error_byte & 0x02)
                        RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),
                                    "Servo %d: Angle Limit Error", id);
                    if (error_byte & 0x04)
                        RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),
                                    "Servo %d: Overheating Error", id);
                    if (error_byte & 0x08)
                        RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),
                                    "Servo %d: Range Error", id);
                    if (error_byte & 0x10)
                        RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),
                                    "Servo %d: Checksum Error", id);
                    if (error_byte & 0x20)
                        RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),
                                    "Servo %d: Overload Error", id);
                    if (error_byte & 0x40)
                        RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),
                                    "Servo %d: Instruction Error", id);
                    // Don't return - we can still process the data
                }

                // Check if this is a status response (should have at least 8 bytes of data)
                if (packet.size() >= 8)
                {
                    // Extract position (2 bytes, little endian)
                    int16_t raw_pos = static_cast<int16_t>(
                        packet[1] | (packet[2] << 8));

                    // Handle position according to protocol (12-bit resolution)
                    if (raw_pos & (1 << 15))
                    {  // Check sign bit
                        raw_pos = -(raw_pos & ~(1 << 15));
                    }
                    // Convert to radians (4096 counts per revolution)
                    state.position = raw_pos * (2.0 * M_PI / 4096.0);

                    // Extract velocity (2 bytes, little endian)
                    int16_t raw_vel = static_cast<int16_t>(
                        packet[3] | (packet[4] << 8));

                    // Handle velocity according to protocol (-1000 to 1000)
                    if (raw_vel & (1 << 15))
                    {  // Check sign bit
                        raw_vel = -(raw_vel & ~(1 << 15));
                    }
                    // Convert to rad/s (protocol units are roughly RPM/1000)
                    const double rpm = raw_vel * (MAX_RPM / 1000.0);
                    state.velocity = rpm * (2.0 * M_PI / 60.0);

                    // Extract temperature (1 byte)
                    state.temperature = static_cast<double>(packet[7]);

                    RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME),
                                 "Servo %d state updated - Pos: %.2f rad, Vel: %.2f rad/s, Temp: %.1f°C",
                                 id, state.position, state.velocity, state.temperature);
                }
            }

            // Skip to end of this packet
            i += 3 + length;
        }
    }

    void ST3215SystemHardware::scheduleNextRead() noexcept
    {
        _read_timer_.expires_after(READ_TIMEOUT);
        _read_timer_.async_wait([this](const boost::system::error_code& error)
                                {
        if (!error) {
            startAsyncRead();
        } });
    }

    void ST3215SystemHardware::scheduleNextWrite() noexcept
    {
        _write_timer_.expires_after(WRITE_TIMEOUT);
        _write_timer_.async_wait([this](const boost::system::error_code& error)
                                 {
        if (!error) {
            startAsyncWrite();
        } });
    }

    void ST3215SystemHardware::startAsyncRead() noexcept
    {
        RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), "Starting async read operation");
        _serial_port_.async_read_some(
            boost::asio::buffer(_read_buffer_),
            [this](const boost::system::error_code& error, std::size_t bytes_transferred)
            {
                if (error)
                {
                    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                                 "Async read error: %s", error.message().c_str());
                }
                else
                {
                    RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME),
                                 "Async read completed: %zu bytes received", bytes_transferred);
                    // Process the received data
                    for (size_t i = 0; i < std::min(bytes_transferred, size_t(8)); i++)
                    {
                        RCLCPP_DEBUG_STREAM(rclcpp::get_logger(LOGGER_NAME),
                                            std::hex << std::setw(2) << std::setfill('0')
                                                     << static_cast<int>(_read_buffer_[i]) << " ");
                    }
                    processResponse(std::span{_read_buffer_.data(), bytes_transferred});
                }

                // Schedule next read
                RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), "Scheduling next read");
                scheduleNextRead();
            });
    }

    void ST3215SystemHardware::startAsyncWrite() noexcept
    {
        if (_write_queue_.empty())
        {
            RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), "Write queue empty, skipping async write");
            return;
        }

        const auto& packet = _write_queue_.front();
        RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), "Starting async write of %zu bytes", packet.size());

        boost::asio::async_write(
            _serial_port_,
            boost::asio::buffer(packet),
            [this](const boost::system::error_code& error, std::size_t bytes_written)
            {
                if (error)
                {
                    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                                 "Async write error: %s", error.message().c_str());
                }
                else
                {
                    RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME),
                                 "Async write completed: %zu bytes written", bytes_written);
                    std::lock_guard<std::mutex> lock(_serial_mutex_);
                    _write_queue_.pop();
                }

                // Schedule next write
                RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), "Scheduling next write");
                scheduleNextWrite();
            });
    }

    bool ST3215SystemHardware::updateServoStates(uint8_t id, size_t index) noexcept
    {
        try
        {
            // Update the timestamp first
            _last_update_times_[index] = rclcpp::Clock(RCL_ROS_TIME).now();

            _current_positions_[index] = 0.0;   // Replace with actual position reading
            _current_velocities_[index] = 0.0;  // Replace with actual velocity reading
            // _current_loads_[index] = 0.0;       // Replace with actual load reading
            _temperatures_[index] = 25.0;  // Replace with actual temperature reading

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

}  // namespace perseus_lite_hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    perseus_lite_hardware::ST3215SystemHardware,
    hardware_interface::SystemInterface)