#include "space_resources/space_resources_controller/main.hpp"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <memory>
#include <sstream>
//
static std::string iso_timestamp()
{
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::ostringstream ss;
    ss << std::put_time(std::gmtime(&t), "%Y-%m-%dT%H:%M:%SZ");
    return ss.str();
}

SpaceResourcesController::SpaceResourcesController(const rclcpp::NodeOptions& options)
    : rclcpp::Node("space_resources_controller", options)
{
    // Service setup so controller node can call water or illmenite ML 'ask' for concentration to be sent back
    _water_concentration_service =
        this->create_client<perseus_interfaces::srv::Concentration>("/water/concentration");
    _ilmenite_concentration_service =
        this->create_client<perseus_interfaces::srv::Concentration>("/ilmenite/concentration");

    _water_reading_service = this->create_service<perseus_interfaces::srv::Concentration>(
        "/water/reading",
        std::bind(&SpaceResourcesController::_handle_water_reading_request, this,
                  std::placeholders::_1, std::placeholders::_2));

    _ilmenite_reading_service = this->create_service<perseus_interfaces::srv::Concentration>(
        "/ilmenite/reading",
        std::bind(&SpaceResourcesController::_handle_ilmenite_reading_request, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Publishers to send concentration results to WebUI
    _water_concentration_result_pub = this->create_publisher<std_msgs::msg::Float64>(
        "/water_concentration/result", 10);

    _ilmenite_concentration_result_pub = this->create_publisher<std_msgs::msg::Float64>(
        "/ilmenite_concentration/result", 10);

    // MOVE TO WEBUI NODE!!
    // Service setup so the WebUI node can call water or illmenite actions to be triggered by the controller (i.e first step)
    // _water_reading_service =
    //     this->create_client<perseus_interfaces::srv::Concentration>("/water/reading");
    // _ilmenite_reading_service =
    //     this->create_client<perseus_interfaces::srv::Concentration>("/ilmenite/reading");

    // Service call 'receiving' end - when WebUI calls /water/reading or /ilmenite/reading,
    // the corresponding handler triggers the ML pipeline and waits for the result

    RCLCPP_INFO(this->get_logger(), "Space Resources Controller ready.");
}

// MOVE TO ML NODE
// When service is called to trigger the ML for illmenite sensor, this code should be in ML node and have the concentration calculation trigger/procecsses inside it
// void SpaceResourcesController::_reading_request_callback(
//     const std::shared_ptr<perseus_interfaces::srv::Concentration::Request> request,
//     std::shared_ptr<perseus_interfaces::srv::Concentration::Response> response)
// {
//     // read a sensor, calculate concentration
//     response->concentration = 0.42;  // placeholder for the response
// }

// Called when WebUI triggers /water/reading — forwards to ML node
void SpaceResourcesController::_handle_water_reading_request(
    const std::shared_ptr<perseus_interfaces::srv::Concentration::Request> request,
    std::shared_ptr<perseus_interfaces::srv::Concentration::Response> response)
{
    _call_concentration_service(_water_concentration_service, "water");
}

// Called when WebUI triggers /ilmenite/reading — forwards to ML node
void SpaceResourcesController::_handle_ilmenite_reading_request(
    const std::shared_ptr<perseus_interfaces::srv::Concentration::Request> request,
    std::shared_ptr<perseus_interfaces::srv::Concentration::Response> response)
{
    _call_concentration_service(_ilmenite_concentration_service, "ilmenite");
}

void SpaceResourcesController::_call_concentration_service(
    rclcpp::Client<perseus_interfaces::srv::Concentration>::SharedPtr client,
    const std::string& sample_type)
{
    if (!client->wait_for_service(std::chrono::seconds(2)))
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Concentration service for '%s' not available", sample_type.c_str());
        return;
    }
    // Sending 'call' to the ML, triggering the process that will send the concentration back to the controller
    auto request = std::make_shared<perseus_interfaces::srv::Concentration::Request>();
    client->async_send_request(
        request,
        [this, sample_type](
            rclcpp::Client<perseus_interfaces::srv::Concentration>::SharedFuture future)
        {
            auto response = future.get();

            // Store raw sensor data on the controller for later processing/conversion to JSON
            auto raw_sensor_data = response->illuminance_data;
            _raw_sensor_data = raw_sensor_data;

            // Extract concentration and forward to WebUI via topic
            double concentration = response->concentration;
            std_msgs::msg::Float64 result_msg;
            result_msg.data = concentration;

            if (sample_type.c_str() == "water")
            {
                _water_concentration_result_pub->publish(result_msg);
            }
            else if (sample_type.c_str() == "ilmenite")
            {
                _ilmenite_concentration_result_pub->publish(result_msg);
            }

            RCLCPP_INFO(this->get_logger(),
                        "[%s] concentration=%.4f", sample_type.c_str(), concentration);
        });
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<SpaceResourcesController>();
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"),
                     "Error running Space Resources Controller node: %s", e.what());
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}