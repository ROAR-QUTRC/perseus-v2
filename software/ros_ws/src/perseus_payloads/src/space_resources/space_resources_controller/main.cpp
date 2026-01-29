#include "space_resources/space_resources_controller/main.hpp"

#include <memory>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SpaceResourcesController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
