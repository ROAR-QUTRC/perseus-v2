#include "st3215_system.hpp"

#include <algorithm>
#include <cmath>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

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
        if (serial_port_.is_open())
        {
            serial_port_.close();
        }

        if (io_thread_.joinable())
        {
            io_context_.stop();
            io_thread_.join();
        }
    }

    hardware_interface::CallbackReturn ST3215SystemHardware::on_init(
        const hardware_interface::HardwareInfo& info)
    {
        if (const auto result = SystemInterface::on_init(info);
            result != hardware_interface::CallbackReturn::SUCCESS)
        {
            return result;
        }

        // Validate required parameters
        if (!info.hardware_parameters.contains("serial_port") ||
            !info.hardware_parameters.contains("baud_rate"))
        {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                         "Missing required serial port parameters!");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Pre-allocate vectors with known size
        const auto joint_count = info.joints.size();
        command_positions_.resize(joint_count);
        command_speeds_.resize(joint_count);
        current_positions_.resize(joint_count);
        current_velocities_.resize(joint_count);
        current_loads_.resize(joint_count);
        temperatures_.resize(joint_count, 25.0);  // Initialize to room temperature
        servo_ids_.reserve(joint_count);

        // Extract and validate servo IDs
        for (const auto& joint : info.joints)
        {
            if (!joint.parameters.contains("id"))
            {
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                             "Joint %s missing servo ID!", joint.name.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }

            try
            {
                const auto id = static_cast<uint8_t>(
                    std::stoi(joint.parameters.at("id")));
                servo_ids_.push_back(id);
            }
            catch (const std::exception& e)
            {
                RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                             "Invalid servo ID for joint %s: %s",
                             joint.name.c_str(), e.what());
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface>
    ST3215SystemHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        state_interfaces.reserve(info_.joints.size() * 4);  // 4 interfaces per joint

        for (size_t i = 0; i < servo_ids_.size(); ++i)
        {
            state_interfaces.emplace_back(
                info_.joints[i].name,
                hardware_interface::HW_IF_POSITION,
                &current_positions_[i]);

            state_interfaces.emplace_back(
                info_.joints[i].name,
                hardware_interface::HW_IF_VELOCITY,
                &current_velocities_[i]);

            state_interfaces.emplace_back(
                info_.joints[i].name,
                "load",
                &current_loads_[i]);

            // Custom temperature interface (fixed from HW_IF_TEMPERATURE)
            state_interfaces.emplace_back(
                info_.joints[i].name,
                "temperature",
                &temperatures_[i]);
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface>
    ST3215SystemHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        command_interfaces.reserve(info_.joints.size() * 2);  // 2 interfaces per joint

        for (size_t i = 0; i < servo_ids_.size(); ++i)
        {
            command_interfaces.emplace_back(
                info_.joints[i].name,
                hardware_interface::HW_IF_VELOCITY,
                &command_speeds_[i]);

            command_interfaces.emplace_back(
                info_.joints[i].name,
                hardware_interface::HW_IF_POSITION,
                &command_positions_[i]);
        }

        return command_interfaces;
    }

    hardware_interface::CallbackReturn ST3215SystemHardware::on_configure(
        const rclcpp_lifecycle::State&)
    {
        try
        {
            const auto& serial_port = info_.hardware_parameters.at("serial_port");
            const int baud_rate = std::stoi(info_.hardware_parameters.at("baud_rate"));

            serial_port_.open(serial_port);

            // Configure serial port settings
            serial_port_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
            serial_port_.set_option(boost::asio::serial_port_base::character_size(8));
            serial_port_.set_option(boost::asio::serial_port_base::stop_bits(
                boost::asio::serial_port_base::stop_bits::one));
            serial_port_.set_option(boost::asio::serial_port_base::parity(
                boost::asio::serial_port_base::parity::none));
            serial_port_.set_option(boost::asio::serial_port_base::flow_control(
                boost::asio::serial_port_base::flow_control::none));

            // Start IO thread
            io_thread_ = std::thread([this]
                                     { io_context_.run(); });
            startAsyncRead();

            return hardware_interface::CallbackReturn::SUCCESS;
        }
        catch (const boost::system::system_error& e)
        {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                         "Serial port error: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    hardware_interface::CallbackReturn ST3215SystemHardware::on_cleanup(
        const rclcpp_lifecycle::State&)
    {
        if (serial_port_.is_open())
        {
            serial_port_.close();
        }

        if (io_thread_.joinable())
        {
            io_context_.stop();
            io_thread_.join();
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ST3215SystemHardware::on_activate(
        const rclcpp_lifecycle::State&)
    {
        // Reset command positions to current positions to avoid sudden movements
        std::fill(command_positions_.begin(), command_positions_.end(), 0.0);
        std::fill(command_speeds_.begin(), command_speeds_.end(), 0.0);
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ST3215SystemHardware::on_deactivate(
        const rclcpp_lifecycle::State&)
    {
        // Stop all servos
        std::fill(command_speeds_.begin(), command_speeds_.end(), 0.0);
        auto result = write(rclcpp::Time(), rclcpp::Duration::from_seconds(0));
        return (result == hardware_interface::return_type::OK)
                   ? hardware_interface::CallbackReturn::SUCCESS
                   : hardware_interface::CallbackReturn::ERROR;
    }

    hardware_interface::return_type ST3215SystemHardware::read(
        const rclcpp::Time&, const rclcpp::Duration&)
    {
        std::lock_guard<std::mutex> lock(serial_mutex_);

        for (size_t i = 0; i < servo_ids_.size(); ++i)
        {
            // Request servo status for each servo
            if (!sendServoCommand(servo_ids_[i], 0x02, {}))  // 0x02 is typically a status read command
            {
                RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),
                            "Failed to request status for servo %d", servo_ids_[i]);
            }
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type ST3215SystemHardware::write(
        const rclcpp::Time&, const rclcpp::Duration&)
    {
        std::lock_guard<std::mutex> lock(serial_mutex_);

        for (size_t i = 0; i < servo_ids_.size(); ++i)
        {
            // Convert velocity command to servo units (RPM)
            const double rpm = command_speeds_[i] * (60.0 / (2.0 * M_PI));
            const auto servo_speed = static_cast<int16_t>(
                std::clamp(rpm, static_cast<double>(-MAX_VELOCITY_RPM),
                           static_cast<double>(MAX_VELOCITY_RPM)));

            const std::array vel_data{
                static_cast<uint8_t>(servo_speed & 0xFF),
                static_cast<uint8_t>((servo_speed >> 8) & 0xFF)};

            if (!sendServoCommand(servo_ids_[i], CMD_WRITE_VEL,
                                  std::span{vel_data}))
            {
                RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),
                            "Failed to send velocity command to servo %d",
                            servo_ids_[i]);
                return hardware_interface::return_type::ERROR;
            }
        }

        return hardware_interface::return_type::OK;
    }

    bool ST3215SystemHardware::sendServoCommand(
        const uint8_t id, const uint8_t cmd,
        const std::span<const uint8_t> data) noexcept
    {
        if (!serial_port_.is_open())
        {
            return false;
        }

        try
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

            // Queue packet for sending
            std::lock_guard<std::mutex> lock(serial_mutex_);
            write_queue_.emplace(std::move(packet));

            // Start async write if not already running
            if (write_queue_.size() == 1)
            {
                startAsyncWrite();
            }

            return true;
        }
        catch (const std::exception&)
        {
            return false;
        }
    }

    void ST3215SystemHardware::processResponse(
        const std::span<const uint8_t> response) noexcept
    {
        if (response.size() < 4)
        {
            return;
        }

        for (size_t i = 0; i < response.size() - 3; ++i)
        {
            if (response[i] == 0xFF && response[i + 1] == 0xFF)
            {
                const uint8_t id = response[i + 2];
                const uint8_t length = response[i + 3];

                if (i + 4 + length <= response.size())
                {
                    const std::span packet{
                        response.data() + i + 4,
                        static_cast<size_t>(length)};

                    if (const auto it = std::find(servo_ids_.begin(),
                                                  servo_ids_.end(), id);
                        it != servo_ids_.end())
                    {
                        const auto index = std::distance(servo_ids_.begin(), it);
                        if (!updateServoStates(id, index))
                        {
                            RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),
                                        "Failed to update state for servo %d", id);
                        }
                    }
                }
            }
        }
    }

    void ST3215SystemHardware::scheduleNextRead() noexcept
    {
        read_timer_.expires_after(READ_TIMEOUT);
        read_timer_.async_wait([this](const boost::system::error_code& error)
                               {
        if (!error) {
            startAsyncRead();
        } });
    }

    void ST3215SystemHardware::scheduleNextWrite() noexcept
    {
        write_timer_.expires_after(WRITE_TIMEOUT);
        write_timer_.async_wait([this](const boost::system::error_code& error)
                                {
        if (!error) {
            startAsyncWrite();
        } });
    }

    void ST3215SystemHardware::startAsyncRead() noexcept
    {
        serial_port_.async_read_some(
            boost::asio::buffer(read_buffer_),
            [this](const boost::system::error_code& error, std::size_t bytes_transferred)
            {
                if (!error)
                {
                    // Process the received data
                    processResponse(std::span{read_buffer_.data(), bytes_transferred});
                }

                // Schedule next read
                scheduleNextRead();
            });
    }

    void ST3215SystemHardware::startAsyncWrite() noexcept
    {
        if (write_queue_.empty())
        {
            return;
        }

        const auto& packet = write_queue_.front();

        boost::asio::async_write(
            serial_port_,
            boost::asio::buffer(packet),
            [this](const boost::system::error_code& error, std::size_t)
            {
                if (!error)
                {
                    std::lock_guard<std::mutex> lock(serial_mutex_);
                    write_queue_.pop();
                }

                // Schedule next write
                scheduleNextWrite();
            });
    }

    bool ST3215SystemHardware::updateServoStates(uint8_t id, size_t index) noexcept
    {
        try
        {
            // This method would parse the servo response and update the corresponding states
            // For the ST3215 servo, you might need to implement specific parsing logic

            // Example placeholder implementation
            current_positions_[index] = 0.0;   // Replace with actual position reading
            current_velocities_[index] = 0.0;  // Replace with actual velocity reading
            current_loads_[index] = 0.0;       // Replace with actual load reading
            temperatures_[index] = 25.0;       // Replace with actual temperature reading

            return true;
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                         "Error updating servo %d states: %s", id, e.what());
            return false;
        }
    }

}  // namespace perseus_lite_hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    perseus_lite_hardware::ST3215SystemHardware,
    hardware_interface::SystemInterface)