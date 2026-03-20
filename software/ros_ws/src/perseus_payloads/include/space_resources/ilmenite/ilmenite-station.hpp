#pragma once

#include <memory>
#include <perseus_interfaces/srv/get_spectral_data.hpp>
#include <perseus_interfaces/srv/take_as7343_reading.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/rclcpp/service.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>

class IlmeniteStation : public rclcpp::Node
{
public:
    explicit IlmeniteStation(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    ~IlmeniteStation();

private:
    rclcpp::Client<perseus_interfaces::srv::GetSpectralData>::SharedPtr _spectral_data_client;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr _led_client;
    rclcpp::Service<perseus_interfaces::srv::TakeAs7343Reading>::SharedPtr _read_ilmenite_service;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr _ilmenite_set_lid_servo;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr _ilmenite_set_tip_servo;

    uint8_t _lid_servo_pin;
    uint8_t _tip_servo_pin;
    uint8_t _uv_led_pin;

    const uint16_t _lid_open_pulsewidth = 1000;
    const uint16_t _lid_closed_pulsewidth = 2000;
    const uint16_t _tip_upright_pulsewidth = 1000;
    const uint16_t _tip_upside_down_pulsewidth = 2000;

    void _process_ilmenite(const std::shared_ptr<perseus_interfaces::srv::TakeAs7343Reading::Request> request, std::shared_ptr<perseus_interfaces::srv::TakeAs7343Reading::Response> response);
    void _set_lid_servo(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    void _set_tip_servo(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);
};
