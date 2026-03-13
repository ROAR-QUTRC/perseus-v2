/**
 * kibisis_sensors_node.cpp
 *
 * Standalone ROS2 node that provides a clean interface to the Kibisis
 * space motor and moisture sensor. Talks to the hardware exclusively
 * through ros2_control GPIO interfaces — never touches I2C directly.
 *
 * All topics and services are under the /kibisis namespace, set via
 * the node's namespace in the launch file.
 *
 * ROS2 API:
 *
 *   Subscriptions:
 *     ~/space_motor/cmd  (std_msgs/Float32)
 *       Speed -100.0 to 100.0. Published to space_motor_controller.
 *
 *   Services:
 *     ~/moisture/sample  (std_srvs/Trigger)
 *       Request a moisture sample. Result published on ~/moisture/value.
 *
 *   Publishers:
 *     ~/moisture/value  (std_msgs/Float32)
 *       Raw ADC 0.0-4095.0, published after each completed sample.
 */

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class KibisisSensorsNode : public rclcpp::Node
{
public:
    KibisisSensorsNode() : Node("kibisis_sensors")
    {
        // Publishes moisture ADC result after a sample completes
        moisture_pub_ = create_publisher<std_msgs::msg::Float32>("moisture/value", 10);

        // Subscribes to space motor speed commands from user code,
        // forwards them to the ForwardCommandController topic
        space_motor_sub_ = create_subscription<std_msgs::msg::Float32>(
            "space_motor/cmd", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                // Clamp to valid range before forwarding
                auto clamped = std_msgs::msg::Float32();
                clamped.data = std::clamp(msg->data, -100.0f, 100.0f);
                space_motor_cmd_pub_->publish(clamped);
            });

        // Forwards space motor commands to the ros2_control
        // ForwardCommandController that owns space_motor/velocity
        space_motor_cmd_pub_ = create_publisher<std_msgs::msg::Float32>(
            "/kibisis/space_motor_controller/commands", 10);

        // Service to trigger a moisture sample
        moisture_service_ = create_service<std_srvs::srv::Trigger>(
            "moisture/sample",
            [this](
                const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
                std_srvs::srv::Trigger::Response::SharedPtr res)
            {
                handle_moisture_sample(res);
            });

        // Sends trigger=1.0 to the ForwardCommandController that owns
        // moisture_sensor/trigger GPIO command interface
        moisture_trigger_pub_ = create_publisher<std_msgs::msg::Float32>(
            "/kibisis/moisture_trigger_controller/commands", 10);

        // Reads moisture ADC value broadcast by JointStateBroadcaster
        // (which broadcasts all GPIO state interfaces alongside joints)
        moisture_state_sub_ = create_subscription<std_msgs::msg::Float32>(
            "/kibisis/moisture/adc_raw", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                if (waiting_for_moisture_)
                {
                    moisture_pub_->publish(*msg);
                    waiting_for_moisture_ = false;
                    RCLCPP_INFO(get_logger(),
                        "Moisture sample complete: %.0f / 4095", msg->data);
                }
            });

        RCLCPP_INFO(get_logger(), "kibisis_sensors ready");
        RCLCPP_INFO(get_logger(), "  Space motor cmd:  ~/space_motor/cmd");
        RCLCPP_INFO(get_logger(), "  Moisture service: ~/moisture/sample");
        RCLCPP_INFO(get_logger(), "  Moisture result:  ~/moisture/value");
    }

private:
    void handle_moisture_sample(
        const std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        if (waiting_for_moisture_)
        {
            res->success = false;
            res->message = "Sample already in progress";
            return;
        }

        auto trigger_msg = std_msgs::msg::Float32();
        trigger_msg.data = 1.0f;
        moisture_trigger_pub_->publish(trigger_msg);
        waiting_for_moisture_ = true;

        RCLCPP_INFO(get_logger(), "Moisture sample triggered");
        res->success = true;
        res->message = "Triggered — result on ~/moisture/value";
    }

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr moisture_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr space_motor_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr moisture_trigger_pub_;

    // Subscriptions
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr space_motor_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr moisture_state_sub_;

    // Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr moisture_service_;

    bool waiting_for_moisture_ = false;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KibisisSensorsNode>());
    rclcpp::shutdown();
    return 0;
}
