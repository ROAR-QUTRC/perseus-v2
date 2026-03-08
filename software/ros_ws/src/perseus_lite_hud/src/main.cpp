#include <rclcpp/rclcpp.hpp>

#include "perseus_lite_hud/hud_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<perseus_lite_hud::HudOverlayNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
