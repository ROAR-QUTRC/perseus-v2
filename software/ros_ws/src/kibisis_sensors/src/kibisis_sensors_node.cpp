/**
 * kibisis_sensors_node.cpp
 *
 * Standalone ROS2 node that provides a clean interface to the Kibisis
 * space motor, moisture sensor, and LDR sensors. Talks to the hardware
 * exclusively through ros2_control GPIO interfaces — never touches I2C
 * directly.
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
 *     ~/ldr/sample  (std_srvs/Trigger)
 *       Request an LDR sample. Results published on ~/ldr/a_ambient,
 *       ~/ldr/b_ambient, ~/ldr/a_illuminated, ~/ldr/b_illuminated.
 *
 *   Publishers:
 *     ~/moisture/value         (std_msgs/Float32)  raw ADC 0-4095
 *     ~/ldr/a_ambient          (std_msgs/Float32)  LDR A ambient 0-4095
 *     ~/ldr/b_ambient          (std_msgs/Float32)  LDR B ambient 0-4095
 *     ~/ldr/a_illuminated      (std_msgs/Float32)  LDR A illuminated 0-4095
 *     ~/ldr/b_illuminated      (std_msgs/Float32)  LDR B illuminated 0-4095
 */

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class KibisisSensorsNode : public rclcpp::Node
{
public:
    KibisisSensorsNode()
        : Node("kibisis_sensors")
    {
        // --- Space motor ---
        space_motor_sub_ = create_subscription<std_msgs::msg::Float32>(
            "space_motor/cmd", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg)
            {
                auto clamped = std_msgs::msg::Float32();
                clamped.data = std::clamp(msg->data, -100.0f, 100.0f);
                space_motor_cmd_pub_->publish(clamped);
            });

        space_motor_cmd_pub_ = create_publisher<std_msgs::msg::Float32>(
            "/kibisis/space_motor_controller/commands", 10);

        // --- Moisture sensor ---
        moisture_pub_ = create_publisher<std_msgs::msg::Float32>("moisture/value", 10);

        moisture_service_ = create_service<std_srvs::srv::Trigger>(
            "moisture/sample",
            [this](
                const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
                std_srvs::srv::Trigger::Response::SharedPtr res)
            {
                handle_moisture_sample(res);
            });

        moisture_trigger_pub_ = create_publisher<std_msgs::msg::Float32>(
            "/kibisis/moisture_trigger_controller/commands", 10);

        moisture_state_sub_ = create_subscription<std_msgs::msg::Float32>(
            "/kibisis/moisture/adc_raw", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg)
            {
                if (waiting_for_moisture_)
                {
                    moisture_pub_->publish(*msg);
                    waiting_for_moisture_ = false;
                    RCLCPP_INFO(get_logger(),
                                "Moisture sample complete: %.0f / 4095", msg->data);
                }
            });

        // --- LDR sensors ---
        ldr_a_ambient_pub_ = create_publisher<std_msgs::msg::Float32>("ldr/a_ambient", 10);
        ldr_b_ambient_pub_ = create_publisher<std_msgs::msg::Float32>("ldr/b_ambient", 10);
        ldr_a_illuminated_pub_ = create_publisher<std_msgs::msg::Float32>("ldr/a_illuminated", 10);
        ldr_b_illuminated_pub_ = create_publisher<std_msgs::msg::Float32>("ldr/b_illuminated", 10);

        ldr_service_ = create_service<std_srvs::srv::Trigger>(
            "ldr/sample",
            [this](
                const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
                std_srvs::srv::Trigger::Response::SharedPtr res)
            {
                handle_ldr_sample(res);
            });

        ldr_trigger_pub_ = create_publisher<std_msgs::msg::Float32>(
            "/kibisis/ldr_trigger_controller/commands", 10);

        // Subscribe to all four LDR state interfaces broadcast by JointStateBroadcaster
        ldr_a_ambient_sub_ = create_subscription<std_msgs::msg::Float32>(
            "/kibisis/ldr/a_ambient", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg)
            {
                if (waiting_for_ldr_)
                {
                    ldr_a_ambient_pub_->publish(*msg);
                    ldr_a_received_ = true;
                    check_ldr_complete();
                }
            });

        ldr_b_ambient_sub_ = create_subscription<std_msgs::msg::Float32>(
            "/kibisis/ldr/b_ambient", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg)
            {
                if (waiting_for_ldr_)
                {
                    ldr_b_ambient_pub_->publish(*msg);
                    ldr_b_received_ = true;
                    check_ldr_complete();
                }
            });

        ldr_a_illuminated_sub_ = create_subscription<std_msgs::msg::Float32>(
            "/kibisis/ldr/a_illuminated", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg)
            {
                if (waiting_for_ldr_)
                {
                    ldr_a_illuminated_pub_->publish(*msg);
                    ldr_a_illuminated_received_ = true;
                    check_ldr_complete();
                }
            });

        ldr_b_illuminated_sub_ = create_subscription<std_msgs::msg::Float32>(
            "/kibisis/ldr/b_illuminated", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg)
            {
                if (waiting_for_ldr_)
                {
                    ldr_b_illuminated_pub_->publish(*msg);
                    ldr_b_illuminated_received_ = true;
                    check_ldr_complete();
                }
            });

        RCLCPP_INFO(get_logger(), "kibisis_sensors ready");
        RCLCPP_INFO(get_logger(), "  Space motor cmd:  ~/space_motor/cmd");
        RCLCPP_INFO(get_logger(), "  Moisture service: ~/moisture/sample");
        RCLCPP_INFO(get_logger(), "  Moisture result:  ~/moisture/value");
        RCLCPP_INFO(get_logger(), "  LDR service:      ~/ldr/sample");
        RCLCPP_INFO(get_logger(), "  LDR results:      ~/ldr/[a|b]_[ambient|illuminated]");
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

    void handle_ldr_sample(
        const std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        if (waiting_for_ldr_)
        {
            res->success = false;
            res->message = "LDR sample already in progress";
            return;
        }

        auto trigger_msg = std_msgs::msg::Float32();
        trigger_msg.data = 1.0f;
        ldr_trigger_pub_->publish(trigger_msg);
        waiting_for_ldr_ = true;
        ldr_a_received_ = false;
        ldr_b_received_ = false;
        ldr_a_illuminated_received_ = false;
        ldr_b_illuminated_received_ = false;

        RCLCPP_INFO(get_logger(), "LDR sample triggered");
        res->success = true;
        res->message = "Triggered — results on ~/ldr/[a|b]_[ambient|illuminated]";
    }

    void check_ldr_complete()
    {
        if (ldr_a_received_ && ldr_b_received_ &&
            ldr_a_illuminated_received_ && ldr_b_illuminated_received_)
        {
            waiting_for_ldr_ = false;
            RCLCPP_INFO(get_logger(), "LDR sample complete");
        }
    }

    // Space motor
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr space_motor_cmd_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr space_motor_sub_;

    // Moisture sensor
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr moisture_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr moisture_trigger_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr moisture_state_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr moisture_service_;
    bool waiting_for_moisture_ = false;

    // LDR sensors
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ldr_a_ambient_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ldr_b_ambient_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ldr_a_illuminated_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ldr_b_illuminated_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ldr_trigger_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ldr_a_ambient_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ldr_b_ambient_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ldr_a_illuminated_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ldr_b_illuminated_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ldr_service_;
    bool waiting_for_ldr_ = false;
    bool ldr_a_received_ = false;
    bool ldr_b_received_ = false;
    bool ldr_a_illuminated_received_ = false;
    bool ldr_b_illuminated_received_ = false;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KibisisSensorsNode>());
    rclcpp::shutdown();
    return 0;
}