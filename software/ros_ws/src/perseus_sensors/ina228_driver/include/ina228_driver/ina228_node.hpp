#pragma once

#include <atomic>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>

#include "ina228_driver/i2c_device.hpp"
#include "perseus_interfaces/msg/dc_power_data.hpp"

namespace ina228_driver
{

    class Ina228Node : public rclcpp::Node
    {
    public:
        explicit Ina228Node(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        ~Ina228Node();

    private:
        void _initialize_parameters();
        void _initialize_publishers();
        void _initialize_services();
        bool _initialize_device();
        void _timer_callback();

        // Register read/write helpers
        bool _read_register_16(uint8_t reg, uint16_t& value);
        bool _read_register_24(uint8_t reg, uint32_t& value);
        bool _read_register_40(uint8_t reg, uint64_t& value);
        bool _write_register_16(uint8_t reg, uint16_t value);

        // Measurement reading
        double _read_bus_voltage();
        double _read_shunt_voltage();
        double _read_current();
        double _read_power();
        double _read_energy();
        double _read_charge();
        double _read_die_temperature();

        // Service callback
        void _reset_accumulators_callback(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response);

        // Parameters
        std::string _i2c_bus_path;
        uint8_t _device_address;
        double _update_rate;
        double _shunt_resistance;
        double _max_current;
        bool _required;

        // Computed calibration values
        double _current_lsb;

        // ROS2 components
        std::unique_ptr<I2cDevice> _i2c_device;
        rclcpp::Publisher<perseus_interfaces::msg::DCPowerData>::SharedPtr _publisher;
        rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr _battery_state_publisher;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _reset_service;
        rclcpp::TimerBase::SharedPtr _timer;

        // Status tracking
        std::atomic<bool> _device_initialized{false};
    };

}  // namespace ina228_driver
