#pragma once

#include <atomic>
#include <memory>
#include <perseus_interfaces/msg/flicker_status.hpp>
#include <perseus_interfaces/msg/spectral_data.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <string>

#include "as7343_driver/as7343_device.hpp"

namespace as7343_driver
{

    class As7343Node : public rclcpp::Node
    {
    public:
        explicit As7343Node(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~As7343Node();

    private:
        void _initialize_parameters();
        void _initialize_publishers();
        bool _initialize_device();
        void _timer_callback();
        void _publish_spectral_data();
        void _publish_flicker_status();

        // Node parameters
        std::string _i2c_bus_path;
        uint8_t _device_address;
        double _publish_rate_hz;
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
        rclcpp::Publisher<perseus_interfaces::msg::SpectralData>::SharedPtr _spectral_pub;
        rclcpp::Publisher<perseus_interfaces::msg::FlickerStatus>::SharedPtr _flicker_pub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _integration_time_pub;
        rclcpp::TimerBase::SharedPtr _timer;

        // Status tracking
        std::atomic<bool> _device_initialized{false};
        size_t _sequence_number{0};
    };

}  // namespace as7343_driver
