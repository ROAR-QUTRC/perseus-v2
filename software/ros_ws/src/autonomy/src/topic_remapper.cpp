#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

class TopicRemapper : public rclcpp::Node
{
public:
    TopicRemapper()
        : Node("topic_remapper")
    {
        // Subscribe to /cmd_vel_nav
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_nav", 10,
            std::bind(&TopicRemapper::callback, this, std::placeholders::_1));

        // Publish to /cmd_vel
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    void callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Republish the received message
        publisher_->publish(*msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TopicRemapper>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}