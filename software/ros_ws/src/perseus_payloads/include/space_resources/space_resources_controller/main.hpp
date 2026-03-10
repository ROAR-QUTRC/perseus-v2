#pragma once
#include <perseus_interfaces/srv/concentration.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/illuminance.hpp>
#include <std_msgs/msg/float64.hpp>
// #include <fstream>
#include <string>
#include <vector>

class SpaceResourcesController : public rclcpp::Node
{
public:
    explicit SpaceResourcesController(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    // Handles /water/reading and /ilmenite/reading service calls from WebUI
    void _handle_water_reading_request(
        const std::shared_ptr<perseus_interfaces::srv::Concentration::Request> request,
        std::shared_ptr<perseus_interfaces::srv::Concentration::Response> response);
    void _handle_ilmenite_reading_request(
        const std::shared_ptr<perseus_interfaces::srv::Concentration::Request> request,
        std::shared_ptr<perseus_interfaces::srv::Concentration::Response> response);

    // Forwards reading request to ML node and handles response
    void _call_concentration_service(
        rclcpp::Client<perseus_interfaces::srv::Concentration>::SharedPtr client,
        const std::string& sample_type);

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _concentration_result_pub;
    rclcpp::Client<perseus_interfaces::srv::Concentration>::SharedPtr _water_concentration_service;
    rclcpp::Client<perseus_interfaces::srv::Concentration>::SharedPtr _ilmenite_concentration_service;
    rclcpp::Service<perseus_interfaces::srv::Concentration>::SharedPtr _water_reading_service;
    rclcpp::Service<perseus_interfaces::srv::Concentration>::SharedPtr _ilmenite_reading_service;

    // Raw sensor data stored for later JSON conversion
    std::vector<sensor_msgs::msg::Illuminance> _raw_sensor_data;
};