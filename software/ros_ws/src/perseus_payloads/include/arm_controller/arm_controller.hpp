#include <rclcpp/rclcpp.hpp>

class ArmController : public rclcpp::Node
{
public:
    explicit ArmController(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    void cleanup();
};