#pragma once

#include <atomic>
#include <memory>
#include <perseus_interfaces/srv/get_flicker_status.hpp>
#include <perseus_interfaces/srv/get_spectral_data.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <string>

#include "as7343_driver/as7343_device.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace as7343_driver
{

    class As7343Node : public rclcpp::Node
    {
    public:
        explicit As7343Node(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~As7343Node() = default;

    private:
        void _initialize_parameters();
        void _initialize_services();
        bool _initialize_device();
        void _handle_get_spectral_data(
            const std::shared_ptr<perseus_interfaces::srv::GetSpectralData::Request> request,
            std::shared_ptr<perseus_interfaces::srv::GetSpectralData::Response> response);
        void _handle_get_flicker_status(
            const std::shared_ptr<perseus_interfaces::srv::GetFlickerStatus::Request> request,
            std::shared_ptr<perseus_interfaces::srv::GetFlickerStatus::Response> response);
        void _handle_set_led_status(
            const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
            std::shared_ptr<std_srvs::srv::SetBool::Response> response);

        // Node parameters
        std::string _i2c_bus_path;
        uint8_t _device_address;
        std::string _frame_id;
        bool _required;
        int _retry_count;

        // Sensor configuration
        uint16_t _gain;
        uint8_t _atime;
        uint16_t _astep;
        uint8_t _smux_mode;
        bool _led_enabled;
        uint8_t _led_current_ma;
        bool _flicker_detection_enabled;

        // ROS2 components
        std::unique_ptr<As7343Device> _device;
        rclcpp::Service<perseus_interfaces::srv::GetSpectralData>::SharedPtr _spectral_srv;
        rclcpp::Service<perseus_interfaces::srv::GetFlickerStatus>::SharedPtr _flicker_srv;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr _led_srv;

        // Status tracking
        std::atomic<bool> _device_initialized{false};
    };

}  // namespace as7343_driver
